package com.example.navgraph.ui

import android.app.Activity
import android.graphics.Typeface
import android.os.SystemClock
import android.text.TextUtils
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.RadioButton
import android.widget.RadioGroup
import android.widget.ScrollView
import android.widget.Space
import android.widget.TextView
import com.example.navgraph.R
import com.example.navgraph.estimator.GnssInsValidation
import com.example.navgraph.estimator.ReplayResult
import com.example.navgraph.logging.LoggerStatus
import java.util.Locale

class DashboardScreen(
    private val activity: Activity,
    onStart: () -> Unit,
    onStop: () -> Unit,
    private val onReplay: () -> Unit,
    private val saved: android.os.Bundle? = null,
) {
    val rootView: View
    private val content = FrameLayout(activity)
    private val statusDot = View(activity)
    private val headerStatus = TextView(activity)
    private val startButton = Button(activity)
    private val stopButton = Button(activity)
    private val permissionBanner = TextView(activity)
    private val solutionState = TextView(activity)
    private val coordinates = TextView(activity)
    private val solutionDetail = TextView(activity)
    private val imuMetric = TextView(activity)
    private val gnssMetric = TextView(activity)
    private val fixMetric = TextView(activity)
    private val pipelineState = TextView(activity)
    private val trajectoryView = TrajectoryView(activity)
    private val liveMap = StreetMapPanel(activity, false, saved?.getBundle("liveMap"))
    private val replayMap = StreetMapPanel(activity, true, saved?.getBundle("replayMap"))
    fun save(out: android.os.Bundle) { out.putInt("tab", selectedTab); out.putBundle("liveMap", liveMap.save()); out.putBundle("replayMap", replayMap.save()) }
    fun onStart() { liveMap.start(); replayMap.start() }
    fun onStop() { liveMap.stop(); replayMap.stop() }
    fun onDestroy() { liveMap.destroy(); replayMap.destroy() }
    fun onLowMemory() { liveMap.lowMemory(); replayMap.lowMemory() }
    private val diagnosticImu = TextView(activity)
    private val diagnosticGnss = TextView(activity)
    private val diagnosticClock = TextView(activity)
    private val diagnosticLog = TextView(activity)
    private val diagnosticLive = TextView(activity)
    private val diagnosticQueue = TextView(activity)
    private val diagnosticNavigation = TextView(activity)
    private val diagnosticEphemeris = TextView(activity)
    private val diagnosticEstimator = TextView(activity)
    private val diagnosticIns = TextView(activity)
    private val diagnosticSmoother = TextView(activity)
    private val diagnosticImuPairing = TextView(activity)
    private var selectedTab = saved?.getInt("tab", 0) ?: 0
    private var status = LoggerStatus()
    private var permissionsGranted = false
    private var replayResult: ReplayResult? = ReplayDisplayCache.result
    private var replayRunning = false
    private var replayFailure: String? = null

    init {
        val root = LinearLayout(activity).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(activity.getColor(R.color.canvas))
            setPadding(dp(16), dp(10), dp(16), dp(12))
        }
        root.setOnApplyWindowInsetsListener { view, insets ->
            val bars = insets.getInsets(android.view.WindowInsets.Type.systemBars() or android.view.WindowInsets.Type.displayCutout())
            view.setPadding(dp(16) + bars.left, dp(10) + bars.top, dp(16) + bars.right, dp(12) + bars.bottom)
            insets
        }
        root.addView(buildHeader(), LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT)
        root.addView(buildTabs(), LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT)
        root.addView(content, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f,
        ))
        startButton.setOnClickListener { onStart() }
        stopButton.setOnClickListener { onStop() }
        when (selectedTab) { 1 -> showReplay(); 2 -> showDiagnostics(); else -> showLive() }
        rootView = root
    }

    fun setPermissionsGranted(granted: Boolean) {
        permissionsGranted = granted
        permissionBanner.visibility = if (granted) View.GONE else View.VISIBLE
        startButton.isEnabled = !status.logging
        startButton.alpha = if (status.logging) 0.45f else 1f
    }

    fun update(newStatus: LoggerStatus) {
        status = newStatus
        val active = newStatus.logging
        headerStatus.text = if (active) "RECORDING" else "READY"
        statusDot.alpha = if (active) 1f else 0.25f
        startButton.isEnabled = !active
        startButton.alpha = if (active) 0.45f else 1f
        stopButton.isEnabled = active
        stopButton.alpha = if (active) 1f else 0.45f
        val elapsedSeconds = if (active && newStatus.startedElapsedNanos > 0L) {
            (SystemClock.elapsedRealtimeNanos() - newStatus.startedElapsedNanos) / 1.0e9
        } else 0.0
        val imuRate = if (elapsedSeconds > 0.2) newStatus.imuCount / elapsedSeconds else 0.0
        imuMetric.text = String.format(Locale.US, "%.0f /s", imuRate)
        gnssMetric.text = newStatus.gnssMeasurementCount.toString()
        fixMetric.text = newStatus.fusedLocationCount.toString()
        pipelineState.text = when {
            !active -> "Sensor stream idle  •  live estimator ready  •  replay FGO ready"
            newStatus.liveSolutionSource != null ->
                "${newStatus.liveSolutionSource} authoritative  •  " +
                    "${newStatus.liveEkfGnssUpdates} GNSS corrections  •  " +
                    "${newStatus.liveFgoUpdates} delayed FGO"
            newStatus.gnssEpochCount == 0L ->
                "IMU propagation waiting for clean GNSS initialization"
            newStatus.liveCleanGnssEpochs > 0L ->
                "Clean GNSS ready  •  ${formatLiveState(newStatus.liveBackendState)}"
            newStatus.liveUnavailableEphemerisCount > 0L ->
                "IMU stream active  •  GNSS waiting for ephemeris  •  replay FGO ready"
            else -> "IMU stream active  •  GNSS preprocessing  •  replay FGO ready"
        }
        if (newStatus.liveSolutionLatitude != null && newStatus.liveSolutionLongitude != null) {
            val solutionAgeMillis = newStatus.liveSolutionElapsedRealtimeNanos?.let {
                maxOf(SystemClock.elapsedRealtimeNanos() - it, 0L) / 1.0e6
            }
            coordinates.text = String.format(
                Locale.US,
                "%.7f\n%.7f",
                newStatus.liveSolutionLatitude,
                newStatus.liveSolutionLongitude,
            )
            solutionState.text = "LIVE ${newStatus.liveSolutionSource ?: "ESTIMATOR"} SOLUTION"
            val sigma = newStatus.liveSolutionHorizontalSigmaMeters
                ?.takeIf(Double::isFinite)
                ?.let { "σh ${format(it, 1)} m" }
                ?: "covariance pending"
            val age = solutionAgeMillis?.let { "age ${format(it, 0)} ms" } ?: "age n/a"
            solutionDetail.text = "$sigma  •  $age  •  h ${format(newStatus.liveSolutionAltitudeMeters ?: 0.0, 1)} m"
            trajectoryView.addLocation(
                newStatus.liveSolutionLatitude,
                newStatus.liveSolutionLongitude,
            )
        } else if (newStatus.latitude != null && newStatus.longitude != null) {
            coordinates.text = String.format(
                Locale.US, "%.7f\n%.7f", newStatus.latitude, newStatus.longitude,
            )
            solutionState.text = "ANDROID BASELINE FIX"
            solutionDetail.text = String.format(
                Locale.US, "± %.1f m  •  h %.1f m", newStatus.accuracyMeters ?: 0f,
                newStatus.altitudeMeters ?: 0.0,
            )
            trajectoryView.addLocation(newStatus.latitude, newStatus.longitude)
        } else {
            coordinates.text = "—\n—"
            solutionState.text = if (active) "WAITING FOR FIRST FIX" else "SESSION NOT STARTED"
            solutionDetail.text = "Android fused baseline"
        }
        diagnosticImu.text = "${newStatus.imuCount} events  •  " +
            "${String.format(Locale.US, "%.1f", imuRate)} /s total"
        diagnosticGnss.text = "${newStatus.gnssEpochCount} epochs  •  ${newStatus.gnssMeasurementCount} measurements"
        diagnosticClock.text = "Hardware discontinuity  ${newStatus.clockDiscontinuityCount}"
        diagnosticLog.text = newStatus.logName.ifBlank { "No session file" }
        diagnosticLive.text = "${formatLiveState(newStatus.liveEstimatorState)}  •  " +
            "${newStatus.liveCleanGnssEpochs} clean epochs"
        diagnosticQueue.text = "${newStatus.liveProcessedEvents} processed  •  " +
            "${newStatus.liveQueuedEvents} queued  •  ${newStatus.liveDroppedEvents} dropped"
        diagnosticNavigation.text = "${newStatus.navigationMessageCount} messages  •  " +
            "${newStatus.completeEphemerides} complete GPS"
        diagnosticEphemeris.text = if (newStatus.completeEphemerides > 0) {
            "GPS L1 C/A broadcast  •  ${newStatus.rejectedNavigationMessages} rejected"
        } else {
            "GPS L1 C/A waiting  •  ${newStatus.rejectedNavigationMessages} rejected"
        }
        diagnosticEstimator.text = "WLS ${compactEngine(newStatus.liveWlsEngine)}  " +
            "${newStatus.liveWlsUpdates}/${newStatus.liveWlsFailures}  " +
            "f${newStatus.liveNativeWlsFailures}\n" +
            "EKF ${compactEngine(newStatus.liveEkfEngine)}  " +
            "p${newStatus.liveNativeEkfPropagationUpdates}  " +
            "g${newStatus.liveNativeEkfGnssUpdates}  f${newStatus.liveNativeEkfFailures}"
        diagnosticIns.text = "${newStatus.liveInsEngine}  •  " +
            "${newStatus.liveNativeInsPropagationUpdates} native  •  " +
            "${newStatus.liveNativeInsFailures} fallback"
        diagnosticSmoother.text = "${compactEngine(newStatus.liveFgoEngine)}  " +
            "${newStatus.liveFgoUpdates} graph  n${newStatus.liveNativeFgoWindowUpdates}  " +
            "f${newStatus.liveNativeFgoFailures}\n" +
            "${newStatus.liveFgoFeedbackAccepted}/${newStatus.liveFgoFeedbackRejected} fb  •  " +
            "${format(newStatus.liveFgoFeedbackCorrectionMeters, 2)} m  •  " +
            formatLiveState(newStatus.liveFgoFeedbackReason)
        diagnosticImuPairing.text = "${newStatus.livePairedImuSamples} paired  •  " +
            "${newStatus.liveUnpairedImuSamples} unmatched  •  " +
            "${format(newStatus.liveMaximumImuPairSkewMillis, 2)} ms max skew"
    }

    private fun compactEngine(engine: String): String = when {
        engine.contains("C++") && engine.contains("READY") -> "C++ READY"
        engine.contains("C++") -> "C++"
        engine.contains("FALLBACK") -> "KOTLIN FB"
        else -> engine
    }

    private fun buildHeader(): View {
        return LinearLayout(activity).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            addView(LinearLayout(activity).apply {
                orientation = LinearLayout.VERTICAL
                addView(text("NAVGRAPH", 20f, R.color.ink, Typeface.BOLD))
                addView(text("GNSS / INS / FGO research console", 12f, R.color.muted))
            }, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
            addView(statusDot.apply {
                setBackgroundResource(R.drawable.status_live)
            }, LinearLayout.LayoutParams(dp(9), dp(9)).apply { marginEnd = dp(7) })
            addView(headerStatus.apply {
                text = "READY"
                setTextColor(activity.getColor(R.color.muted))
                textSize = 11f
                typeface = Typeface.DEFAULT_BOLD
            })
        }
    }

    private fun buildTabs(): View {
        return RadioGroup(activity).apply {
            orientation = RadioGroup.HORIZONTAL
            setBackgroundResource(R.drawable.bg_tabs)
            val labels = listOf("Live", "Replay", "Diagnostics")
            labels.forEachIndexed { index, label ->
                addView(RadioButton(activity).apply {
                    id = View.generateViewId()
                    text = label
                    isChecked = index == selectedTab
                    gravity = Gravity.CENTER
                    textSize = 14f
                    minHeight = dp(48)
                    setTextColor(activity.getColor(R.color.ink))
                    typeface = Typeface.DEFAULT_BOLD
                    buttonDrawable = null
                    setBackgroundResource(R.drawable.bg_tab_selector)
                    setPadding(dp(4), 0, dp(4), 0)
                    setOnClickListener {
                        selectedTab = index
                        when (index) {
                            0 -> showLive()
                            1 -> {
                                showReplay()
                                if (replayResult == null && !replayRunning) onReplay()
                            }
                            else -> showDiagnostics()
                        }
                    }
                }, RadioGroup.LayoutParams(0, RadioGroup.LayoutParams.WRAP_CONTENT, 1f))
            }
            check(getChildAt(selectedTab).id)
        }
    }

    private fun showLive() {
        content.removeAllViews()
        val body = LinearLayout(activity).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(0, dp(14), 0, dp(12))
        }
        permissionBanner.apply {
            text = "Location and notification access required"
            setTextColor(activity.getColor(R.color.red))
            textSize = 13f
            setPadding(dp(14), dp(10), dp(14), dp(10))
            setBackgroundResource(R.drawable.bg_panel)
        }
        body.addView(detached(permissionBanner), matchWrap().apply { bottomMargin = dp(10) })
        body.addView(detached(liveMap), matchWrap())
        body.addView(buildActions())
        val diagnostics = LinearLayout(activity).apply {
            orientation = LinearLayout.VERTICAL
            visibility = View.GONE
            addView(buildSolutionPanel(), matchWrap())
            addView(buildMetricsPanel(), matchWrap())
            addView(buildTrajectoryPanel(), matchWrap())
            addView(detached(pipelineState), matchWrap())
        }
        body.addView(Button(activity).apply {
            text = "Show / hide diagnostics"; minHeight = dp(48)
            setOnClickListener { diagnostics.visibility = if (diagnostics.visibility == View.GONE) View.VISIBLE else View.GONE }
        })
        body.addView(diagnostics)
        content.addView(ScrollView(activity).apply {
            isFillViewport = true
            addView(body)
        })
        setPermissionsGranted(permissionsGranted)
        update(status)
    }

    private fun buildSolutionPanel(): View = LinearLayout(activity).apply {
        orientation = LinearLayout.VERTICAL
        setBackgroundResource(R.drawable.bg_panel)
        setPadding(dp(16), dp(14), dp(16), dp(15))
        addView(detached(solutionState).apply {
            text = "SESSION NOT STARTED"
            textSize = 11f
            typeface = Typeface.DEFAULT_BOLD
            setTextColor(activity.getColor(R.color.teal))
        })
        addView(detached(coordinates).apply {
            text = "—\n—"
            textSize = 25f
            typeface = Typeface.MONOSPACE
            setTextColor(activity.getColor(R.color.ink))
            setPadding(0, dp(7), 0, dp(5))
            setLineSpacing(0f, 0.95f)
        })
        addView(detached(solutionDetail).apply {
            text = "Android fused baseline"
            textSize = 13f
            setTextColor(activity.getColor(R.color.muted))
        })
    }

    private fun buildMetricsPanel(): View = LinearLayout(activity).apply {
        orientation = LinearLayout.HORIZONTAL
        setBackgroundResource(R.drawable.bg_panel)
        setPadding(dp(10), dp(13), dp(10), dp(13))
        addView(metric("IMU EVENTS", imuMetric.apply { text = "0 /s" }), weighted())
        addView(divider(), LinearLayout.LayoutParams(dp(1), dp(42)))
        addView(metric("RAW OBS", gnssMetric.apply { text = "0" }), weighted())
        addView(divider(), LinearLayout.LayoutParams(dp(1), dp(42)))
        addView(metric("FIXES", fixMetric.apply { text = "0" }), weighted())
    }

    private fun buildTrajectoryPanel(): View = LinearLayout(activity).apply {
        orientation = LinearLayout.VERTICAL
        setBackgroundResource(R.drawable.bg_panel)
        addView(LinearLayout(activity).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(14), dp(11), dp(14), dp(9))
            addView(text("LOCAL TRAJECTORY", 11f, R.color.muted, Typeface.BOLD), weighted())
            addView(text("ENU", 11f, R.color.teal, Typeface.BOLD))
        })
        addView(detached(trajectoryView).apply {
            setPadding(dp(2), 0, dp(2), dp(2))
            setBackgroundColor(activity.getColor(R.color.panel))
        }, LinearLayout.LayoutParams.MATCH_PARENT, dp(210))
    }

    private fun buildActions(): View = LinearLayout(activity).apply {
        orientation = LinearLayout.HORIZONTAL
        addView(detached(startButton).apply {
            text = "Start recording"
            textSize = 14f
            setTextColor(activity.getColor(android.R.color.white))
            typeface = Typeface.DEFAULT_BOLD
            setBackgroundResource(R.drawable.bg_primary)
            setCompoundDrawablesWithIntrinsicBounds(android.R.drawable.ic_media_play, 0, 0, 0)
            compoundDrawablePadding = dp(8)
            tooltipText = "Start GNSS and inertial recording"
        }, LinearLayout.LayoutParams(0, dp(52), 1f).apply { marginEnd = dp(6) })
        addView(detached(stopButton).apply {
            text = "Stop and save"
            textSize = 14f
            setTextColor(activity.getColor(R.color.ink))
            typeface = Typeface.DEFAULT_BOLD
            setBackgroundResource(R.drawable.bg_secondary)
            setCompoundDrawablesWithIntrinsicBounds(android.R.drawable.ic_media_pause, 0, 0, 0)
            compoundDrawablePadding = dp(8)
            tooltipText = "Stop recording and close the log"
            isEnabled = false
        }, LinearLayout.LayoutParams(0, dp(52), 1f).apply { marginStart = dp(6) })
    }

    private fun showReplay() {
        content.removeAllViews()
        val result = replayResult
        val body = LinearLayout(activity).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(0, dp(14), 0, dp(12))
            addView(text(
                result?.datasetName?.uppercase(Locale.US) ?: "PUBLIC GSDC REPLAY",
                11f,
                R.color.teal,
                Typeface.BOLD,
            ))
            addView(text("On-device estimator validation", 24f, R.color.ink, Typeface.BOLD).apply {
                setPadding(0, dp(5), 0, dp(4))
            })
            val subtitle = when {
                replayRunning -> "Running robust GNSS optimization off the UI thread..."
                replayFailure != null -> "Replay failed  •  inspect solver status below"
                result != null -> "${result.epochCount} epochs  •  ${result.acceptedObservationCount} accepted observations  •  2 s fixed lag"
                else -> "Public regression fixture  •  ready to run"
            }
            addView(text(subtitle, 13f, if (replayFailure == null) R.color.muted else R.color.red))
            addView(Space(activity), LinearLayout.LayoutParams(1, dp(14)))
            if (result != null) {
                replayMap.setSamples(result.geographicTracks)
                addView(detached(replayMap), matchWrap())
                addView(buildComparisonTable(result), matchWrap().apply { bottomMargin = dp(10) })
                addView(panelSection("SOLVER HEALTH", listOf(
                    "WLS" to "${failureCount(result.wlsMetrics.solverFailures)}   ${format(result.wlsMetrics.meanLatencyMillis, 2)} ms mean",
                    "FGO" to "${failureCount(result.fgoMetrics.solverFailures)}   ${format(result.fgoMetrics.meanLatencyMillis, 2)} ms mean",
                    "Measurements" to result.measurementProfile,
                    "Rejected" to "${result.fgoMetrics.rejectedMeasurements} post-fit   ${result.preprocessingRejections.values.sum()} pre-QC",
                    "Jacobians" to "PR ${scientific(result.pseudorangeJacobianError)}   D ${scientificOrUnavailable(result.dopplerJacobianError)}",
                    "Total replay" to "${format(result.totalComputeLatencyMillis, 1)} ms",
                )), matchWrap().apply { bottomMargin = dp(10) })
                result.gnssInsValidation?.let { validation ->
                    addView(buildGnssInsSection(validation), matchWrap().apply { bottomMargin = dp(10) })
                }
            } else if (replayFailure != null) {
                addView(panelSection("REPLAY ERROR", listOf(
                    "Status" to "Solver stopped",
                    "Reason" to replayFailure.orEmpty(),
                )), matchWrap().apply { bottomMargin = dp(10) })
            } else {
                addView(panelSection("PIPELINE", listOf(
                    "Preprocessing" to "Schema-aware corrections and validity QC",
                    "Baseline" to "Robust pseudorange WLS",
                    "Graph" to "Available GNSS observations + motion factors",
                )), matchWrap().apply { bottomMargin = dp(10) })
            }
            addView(Button(activity).apply {
                text = when {
                    replayRunning -> "Running replay"
                    result == null -> "Run estimator"
                    else -> "Run again"
                }
                isEnabled = !replayRunning
                alpha = if (replayRunning) 0.45f else 1f
                textSize = 14f
                typeface = Typeface.DEFAULT_BOLD
                setTextColor(activity.getColor(android.R.color.white))
                setBackgroundResource(R.drawable.bg_primary)
                setCompoundDrawablesWithIntrinsicBounds(android.R.drawable.ic_media_play, 0, 0, 0)
                compoundDrawablePadding = dp(8)
                tooltipText = "Run the bundled 600-second GSDC replay"
                setOnClickListener { onReplay() }
            }, LinearLayout.LayoutParams.MATCH_PARENT, dp(52))
            addView(Space(activity), LinearLayout.LayoutParams(1, dp(14)))
            addView(text("REGRESSION SCOPE", 11f, R.color.muted, Typeface.BOLD).apply {
                setPadding(dp(2), dp(4), 0, dp(5))
            })
            addView(text(
                result?.datasetScope ?: "600-second public drive segment / on-device compute",
                13f,
                R.color.ink,
            ))
        }
        content.addView(ScrollView(activity).apply { addView(body) })
    }

    fun showReplayLoading() {
        replayRunning = true
        replayFailure = null
        showReplay()
    }

    fun showReplayResult(result: ReplayResult) {
        replayRunning = false
        replayFailure = null
        ReplayDisplayCache.result = result
        replayResult = result
        showReplay()
    }

    fun showReplayFailure(message: String) {
        replayRunning = false
        replayFailure = message
        showReplay()
    }

    private fun buildComparisonTable(result: ReplayResult): View = LinearLayout(activity).apply {
        orientation = LinearLayout.VERTICAL
        setBackgroundResource(R.drawable.bg_panel)
        setPadding(dp(14), dp(13), dp(14), dp(14))
        addView(LinearLayout(activity).apply {
            orientation = LinearLayout.HORIZONTAL
            addView(text("METRIC", 11f, R.color.muted, Typeface.BOLD), weighted(1.25f))
            addView(text("WLS", 11f, R.color.muted, Typeface.BOLD), weighted())
            addView(text("FGO", 11f, R.color.teal, Typeface.BOLD), weighted())
        })
        addView(tableRow("Horizontal P50", "${format(result.wlsMetrics.horizontalP50Meters)} m",
            "${format(result.fgoMetrics.horizontalP50Meters)} m"))
        addView(tableRow("Horizontal P95", "${format(result.wlsMetrics.horizontalP95Meters)} m",
            "${format(result.fgoMetrics.horizontalP95Meters)} m"))
        addView(tableRow("Max jump", "${format(result.wlsMetrics.maxPositionJumpMeters)} m",
            "${format(result.fgoMetrics.maxPositionJumpMeters)} m"))
        addView(tableRow("Smoothness RMS", "${format(result.wlsMetrics.smoothnessRmsMeters)} m",
            "${format(result.fgoMetrics.smoothnessRmsMeters)} m"))
    }

    private fun tableRow(label: String, left: String, right: String): View = LinearLayout(activity).apply {
        orientation = LinearLayout.HORIZONTAL
        gravity = Gravity.CENTER_VERTICAL
        setPadding(0, dp(12), 0, 0)
        addView(text(label, 13f, R.color.ink), weighted(1.25f))
        addView(text(left, 13f, R.color.ink).apply { typeface = Typeface.MONOSPACE }, weighted())
        addView(text(right, 13f, R.color.ink).apply { typeface = Typeface.MONOSPACE }, weighted())
    }

    private fun buildGnssInsSection(validation: GnssInsValidation): View =
        LinearLayout(activity).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(0, dp(8), 0, 0)
            addView(text("SYNTHETIC GNSS / INS", 11f, R.color.blue, Typeface.BOLD))
            addView(text("Bias-aware inertial validation", 20f, R.color.ink, Typeface.BOLD).apply {
                setPadding(0, dp(4), 0, dp(3))
            })
            addView(text(
                "${validation.imuSampleCount} IMU samples  •  ${validation.gnssEpochCount} GNSS epochs  •  " +
                    "${validation.inertialFactorCount} inertial factors",
                13f,
                R.color.muted,
            ))
            addView(Space(activity), LinearLayout.LayoutParams(1, dp(12)))
            addView(buildGnssInsComparisonTable(validation), matchWrap().apply { bottomMargin = dp(10) })
            addView(LinearLayout(activity).apply {
                orientation = LinearLayout.VERTICAL
                setBackgroundResource(R.drawable.bg_panel)
                setPadding(dp(14), dp(12), dp(14), dp(12))
                addView(text("SYNTHETIC LOCAL ENU", 11f, R.color.muted, Typeface.BOLD))
                addView(GnssInsTrajectoryView(activity).apply {
                    setTrajectories(validation)
                }, LinearLayout.LayoutParams.MATCH_PARENT, dp(205))
            }, matchWrap().apply { bottomMargin = dp(10) })
            addView(panelSection("INERTIAL HEALTH", listOf(
                "EKF pseudorange" to "${validation.ekfAcceptedPseudorange} accepted   ${validation.ekfRejectedPseudorange} rejected",
                "EKF Doppler" to "${validation.ekfAcceptedDoppler} accepted   ${validation.ekfRejectedDoppler} rejected",
                "Final position σ" to validation.finalPositionSigmaMeters.joinToString(" / ") {
                    "${format(it, 2)} m"
                },
                "FGO" to "${validation.inertialFactorCount} IMU factors   ${validation.fgoMetrics.solverFailures} failures",
                "Jacobians" to "PR ${scientific(validation.pseudorangeJacobianError)}   D ${scientific(validation.dopplerJacobianError)}",
            )), matchWrap().apply { bottomMargin = dp(8) })
            addView(text(
                "Deterministic sensor-bias test  •  not field accuracy",
                13f,
                R.color.ink,
            ).apply { setPadding(dp(2), 0, 0, dp(4)) })
        }

    private fun buildGnssInsComparisonTable(validation: GnssInsValidation): View =
        LinearLayout(activity).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundResource(R.drawable.bg_panel)
            setPadding(dp(14), dp(13), dp(14), dp(14))
            addView(LinearLayout(activity).apply {
                orientation = LinearLayout.HORIZONTAL
                addView(text("METRIC", 10f, R.color.muted, Typeface.BOLD), weighted(1.35f))
                addView(text("WLS", 10f, R.color.muted, Typeface.BOLD), weighted())
                addView(text("EKF", 10f, R.color.blue, Typeface.BOLD), weighted())
                addView(text("FGO", 10f, R.color.teal, Typeface.BOLD), weighted())
            })
            addView(insTableRow(
                "Horizontal P50",
                validation.wlsMetrics.horizontalP50Meters,
                validation.ekfMetrics.horizontalP50Meters,
                validation.fgoMetrics.horizontalP50Meters,
            ))
            addView(insTableRow(
                "Horizontal P95",
                validation.wlsMetrics.horizontalP95Meters,
                validation.ekfMetrics.horizontalP95Meters,
                validation.fgoMetrics.horizontalP95Meters,
            ))
            addView(insTableRow(
                "Smoothness RMS",
                validation.wlsMetrics.smoothnessRmsMeters,
                validation.ekfMetrics.smoothnessRmsMeters,
                validation.fgoMetrics.smoothnessRmsMeters,
            ))
            addView(insTableRow(
                "Mean latency",
                validation.wlsMetrics.meanLatencyMillis,
                validation.ekfMetrics.meanLatencyMillis,
                validation.fgoMetrics.meanLatencyMillis,
                "ms",
            ))
        }

    private fun insTableRow(
        label: String,
        wls: Double,
        ekf: Double,
        fgo: Double,
        unit: String = "m",
    ): View = LinearLayout(activity).apply {
        orientation = LinearLayout.HORIZONTAL
        gravity = Gravity.CENTER_VERTICAL
        setPadding(0, dp(12), 0, 0)
        addView(text(label, 12f, R.color.ink), weighted(1.35f))
        listOf(wls, ekf, fgo).forEach { value ->
            addView(text("${format(value, 2)} $unit", 12f, R.color.ink).apply {
                typeface = Typeface.MONOSPACE
            }, weighted())
        }
    }

    private fun showDiagnostics() {
        content.removeAllViews()
        val body = LinearLayout(activity).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(0, dp(14), 0, dp(12))
            addView(text("DEVICE PIPELINE", 11f, R.color.teal, Typeface.BOLD))
            addView(text("Timestamp and sensor health", 24f, R.color.ink, Typeface.BOLD).apply {
                setPadding(0, dp(5), 0, dp(14))
            })
            addView(panelSection("INERTIAL", listOf(
                "Accelerometer + gyro" to diagnosticImu,
                "Live pairing" to diagnosticImuPairing,
                "Requested period" to "10,000 µs",
                "Time base" to "elapsedRealtimeNanos",
            )), matchWrap().apply { bottomMargin = dp(10) })
            addView(panelSection("RAW GNSS", listOf(
                "Measurement stream" to diagnosticGnss,
                "Navigation stream" to diagnosticNavigation,
                "Clock state" to diagnosticClock,
                "Tracking request" to "Full tracking",
            )), matchWrap().apply { bottomMargin = dp(10) })
            addView(panelSection("LIVE PREPROCESSOR", listOf(
                "Epoch state" to diagnosticLive,
                "Worker queue" to diagnosticQueue,
                "Ephemeris source" to diagnosticEphemeris,
            )), matchWrap().apply { bottomMargin = dp(10) })
            addView(panelSection("LIVE ESTIMATOR", listOf(
                "WLS / EKF" to diagnosticEstimator,
                "INS propagation" to diagnosticIns,
                "Fixed-lag FGO" to diagnosticSmoother,
            )), matchWrap().apply { bottomMargin = dp(10) })
            addView(panelSection("LOG", listOf(
                "Schema" to "NDJSON v1",
                "Current file" to diagnosticLog,
            )))
        }
        content.addView(ScrollView(activity).apply { addView(body) })
        update(status)
    }

    private fun panelSection(title: String, rows: List<Pair<String, Any>>): View = LinearLayout(activity).apply {
        orientation = LinearLayout.VERTICAL
        setBackgroundResource(R.drawable.bg_panel)
        setPadding(dp(14), dp(13), dp(14), dp(14))
        addView(text(title, 11f, R.color.muted, Typeface.BOLD))
        rows.forEach { (label, value) ->
            addView(LinearLayout(activity).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER_VERTICAL
                setPadding(0, dp(12), 0, 0)
                addView(text(label, 13f, R.color.ink), weighted())
                val valueView = if (value is TextView) detached(value) else text(value.toString(), 13f, R.color.muted)
                valueView.setTextColor(activity.getColor(R.color.muted))
                valueView.textSize = 13f
                valueView.gravity = Gravity.END
                valueView.maxLines = 2
                valueView.ellipsize = TextUtils.TruncateAt.MIDDLE
                addView(valueView, weighted(1.25f))
            })
        }
    }

    private fun metric(label: String, value: TextView): View = LinearLayout(activity).apply {
        orientation = LinearLayout.VERTICAL
        gravity = Gravity.CENTER
        addView(text(label, 10f, R.color.muted, Typeface.BOLD))
        addView(detached(value).apply {
            textSize = 18f
            typeface = Typeface.MONOSPACE
            setTextColor(activity.getColor(R.color.ink))
            setPadding(0, dp(4), 0, 0)
        })
    }

    private fun divider() = View(activity).apply { setBackgroundColor(activity.getColor(R.color.line)) }

    private fun text(value: String, size: Float, color: Int, style: Int = Typeface.NORMAL) =
        TextView(activity).apply {
            text = value
            textSize = size
            setTextColor(activity.getColor(color))
            typeface = Typeface.create(Typeface.DEFAULT, style)
        }

    private fun dp(value: Int): Int = (value * activity.resources.displayMetrics.density).toInt()
    private fun <T : View> detached(view: T): T {
        (view.parent as? ViewGroup)?.removeView(view)
        return view
    }
    private fun format(value: Double, decimals: Int = 2): String =
        String.format(Locale.US, ".${decimals}f".let { "%$it" }, value)

    private fun scientific(value: Double): String = String.format(Locale.US, "%.1e", value)

    private fun scientificOrUnavailable(value: Double): String =
        if (value.isFinite()) scientific(value) else "n/a"

    private fun failureCount(count: Int): String = "$count ${if (count == 1) "failure" else "failures"}"
    private fun formatLiveState(value: String): String = value
        .ifBlank { "idle" }
        .replace('_', ' ')
    private fun weighted(weight: Float = 1f) = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, weight)
    private fun matchWrap() = LinearLayout.LayoutParams(
        LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,
    )
}

private object ReplayDisplayCache { var result: ReplayResult? = null }
