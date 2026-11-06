package com.example.navgraph

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import com.example.navgraph.estimator.AssetReplayEstimator
import com.example.navgraph.estimator.NativeGnssWls
import com.example.navgraph.estimator.NativeGnssInsEkf
import com.example.navgraph.estimator.NativeFixedLagSmoother
import com.example.navgraph.estimator.NativeInsMechanization
import com.example.navgraph.logging.LoggerStatus
import com.example.navgraph.logging.LoggingService
import com.example.navgraph.ui.DashboardScreen
import java.util.concurrent.Executors

class MainActivity : Activity() {
    private lateinit var screen: DashboardScreen
    private var receiverRegistered = false
    private var startAfterPermission = false
    private var replayRunning = false
    private val replayExecutor = Executors.newSingleThreadExecutor()

    private val statusReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent?.action != LoggingService.ACTION_STATUS) return
            val hasLocation = intent.getBooleanExtra(LoggingService.EXTRA_HAS_LOCATION, false)
            val hasLiveSolution = intent.getBooleanExtra(LoggingService.EXTRA_LIVE_HAS_SOLUTION, false)
            val hasFgoSolution = intent.getBooleanExtra(
                LoggingService.EXTRA_LIVE_HAS_FGO_SOLUTION,
                false,
            )
            screen.update(LoggerStatus(
                logging = intent.getBooleanExtra(LoggingService.EXTRA_LOGGING, false),
                sessionId = intent.getStringExtra(LoggingService.EXTRA_SESSION_ID).orEmpty(),
                logName = intent.getStringExtra(LoggingService.EXTRA_LOG_NAME).orEmpty(),
                imuCount = intent.getLongExtra(LoggingService.EXTRA_IMU_COUNT, 0L),
                gnssEpochCount = intent.getLongExtra(LoggingService.EXTRA_GNSS_EPOCH_COUNT, 0L),
                gnssMeasurementCount = intent.getLongExtra(LoggingService.EXTRA_GNSS_MEASUREMENT_COUNT, 0L),
                navigationMessageCount = intent.getLongExtra(
                    LoggingService.EXTRA_NAVIGATION_MESSAGE_COUNT,
                    0L,
                ),
                fusedLocationCount = intent.getLongExtra(LoggingService.EXTRA_LOCATION_COUNT, 0L),
                clockDiscontinuityCount = intent.getIntExtra(LoggingService.EXTRA_CLOCK_DISCONTINUITY, 0),
                latitude = if (hasLocation) intent.getDoubleExtra(LoggingService.EXTRA_LATITUDE, 0.0) else null,
                longitude = if (hasLocation) intent.getDoubleExtra(LoggingService.EXTRA_LONGITUDE, 0.0) else null,
                altitudeMeters = if (hasLocation) intent.getDoubleExtra(LoggingService.EXTRA_ALTITUDE, 0.0) else null,
                accuracyMeters = if (hasLocation) intent.getFloatExtra(LoggingService.EXTRA_ACCURACY, 0f) else null,
                startedElapsedNanos = intent.getLongExtra(LoggingService.EXTRA_STARTED_ELAPSED_NANOS, 0L),
                liveQueuedEvents = intent.getIntExtra(LoggingService.EXTRA_LIVE_QUEUED, 0),
                liveProcessedEvents = intent.getLongExtra(LoggingService.EXTRA_LIVE_PROCESSED, 0L),
                liveDroppedEvents = intent.getLongExtra(LoggingService.EXTRA_LIVE_DROPPED, 0L),
                liveCleanGnssEpochs = intent.getLongExtra(LoggingService.EXTRA_LIVE_CLEAN_GNSS_EPOCHS, 0L),
                liveAcceptedGnssObservations = intent.getLongExtra(LoggingService.EXTRA_LIVE_ACCEPTED_GNSS, 0L),
                liveRejectedGnssObservations = intent.getLongExtra(LoggingService.EXTRA_LIVE_REJECTED_GNSS, 0L),
                liveUnavailableEphemerisCount = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_EPHEMERIS_MISSING,
                    0L,
                ),
                liveEstimatorState = intent.getStringExtra(LoggingService.EXTRA_LIVE_ESTIMATOR_STATE).orEmpty(),
                completeEphemerides = intent.getIntExtra(LoggingService.EXTRA_COMPLETE_EPHEMERIDES, 0),
                rejectedNavigationMessages = intent.getLongExtra(
                    LoggingService.EXTRA_REJECTED_NAVIGATION_MESSAGES,
                    0L,
                ),
                latestEphemerisRejection = intent.getStringExtra(
                    LoggingService.EXTRA_EPHEMERIS_REJECTION,
                ).orEmpty(),
                liveBackendState = intent.getStringExtra(
                    LoggingService.EXTRA_LIVE_BACKEND_STATE,
                ).orEmpty(),
                liveWlsUpdates = intent.getLongExtra(LoggingService.EXTRA_LIVE_WLS_UPDATES, 0L),
                liveWlsFailures = intent.getLongExtra(LoggingService.EXTRA_LIVE_WLS_FAILURES, 0L),
                liveWlsEngine = intent.getStringExtra(LoggingService.EXTRA_LIVE_WLS_ENGINE)
                    ?: NativeGnssWls.readinessLabel,
                liveNativeWlsFailures = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_NATIVE_WLS_FAILURES,
                    0L,
                ),
                liveInsEngine = intent.getStringExtra(LoggingService.EXTRA_LIVE_INS_ENGINE)
                    ?: NativeInsMechanization.readinessLabel,
                liveNativeInsPropagationUpdates = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_NATIVE_INS_PROPAGATIONS,
                    0L,
                ),
                liveNativeInsFailures = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_NATIVE_INS_FAILURES,
                    0L,
                ),
                liveEkfEngine = intent.getStringExtra(LoggingService.EXTRA_LIVE_EKF_ENGINE)
                    ?: NativeGnssInsEkf.readinessLabel,
                liveNativeEkfPropagationUpdates = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_NATIVE_EKF_PROPAGATIONS,
                    0L,
                ),
                liveNativeEkfGnssUpdates = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_NATIVE_EKF_GNSS,
                    0L,
                ),
                liveNativeEkfFailures = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_NATIVE_EKF_FAILURES,
                    0L,
                ),
                liveEkfPropagationUpdates = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_EKF_PROPAGATIONS,
                    0L,
                ),
                liveEkfGnssUpdates = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_EKF_GNSS_UPDATES,
                    0L,
                ),
                liveStaleGnssUpdates = intent.getLongExtra(LoggingService.EXTRA_LIVE_STALE_GNSS, 0L),
                liveFgoUpdates = intent.getLongExtra(LoggingService.EXTRA_LIVE_FGO_UPDATES, 0L),
                liveFgoFailures = intent.getLongExtra(LoggingService.EXTRA_LIVE_FGO_FAILURES, 0L),
                liveFgoEngine = intent.getStringExtra(LoggingService.EXTRA_LIVE_FGO_ENGINE)
                    ?: NativeFixedLagSmoother.readinessLabel,
                liveNativeFgoWindowUpdates = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_NATIVE_FGO_WINDOWS,
                    0L,
                ),
                liveNativeFgoFailures = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_NATIVE_FGO_FAILURES,
                    0L,
                ),
                liveFgoSkippedBusy = intent.getLongExtra(LoggingService.EXTRA_LIVE_FGO_SKIPPED, 0L),
                liveFgoFeedbackAccepted = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_FGO_FEEDBACK_ACCEPTED,
                    0L,
                ),
                liveFgoFeedbackRejected = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_FGO_FEEDBACK_REJECTED,
                    0L,
                ),
                liveFgoFeedbackReason = intent.getStringExtra(
                    LoggingService.EXTRA_LIVE_FGO_FEEDBACK_REASON,
                ) ?: "none",
                liveFgoFeedbackCorrectionMeters = intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_FGO_FEEDBACK_CORRECTION_METERS,
                    0.0,
                ),
                livePairedImuSamples = intent.getLongExtra(LoggingService.EXTRA_LIVE_PAIRED_IMU, 0L),
                liveUnpairedImuSamples = intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_UNPAIRED_IMU,
                    0L,
                ),
                liveMaximumImuPairSkewMillis = intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_IMU_PAIR_SKEW_MS,
                    0.0,
                ),
                liveSolutionSource = if (hasLiveSolution) intent.getStringExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_SOURCE,
                ) else null,
                liveSolutionElapsedRealtimeNanos = if (hasLiveSolution) intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_ELAPSED_NANOS,
                    0L,
                ) else null,
                liveSolutionLatitude = if (hasLiveSolution) intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_LATITUDE,
                    0.0,
                ) else null,
                liveSolutionLongitude = if (hasLiveSolution) intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_LONGITUDE,
                    0.0,
                ) else null,
                liveSolutionAltitudeMeters = if (hasLiveSolution) intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_ALTITUDE,
                    0.0,
                ) else null,
                liveSolutionHorizontalSigmaMeters = if (hasLiveSolution) intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_HORIZONTAL_SIGMA,
                    Double.NaN,
                ) else null,
                liveSolutionVerticalSigmaMeters = if (hasLiveSolution) intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_VERTICAL_SIGMA,
                    Double.NaN,
                ) else null,
                liveSolutionSpeedMetersPerSecond = if (hasLiveSolution) intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_SPEED,
                    Double.NaN,
                ) else null,
                liveSolutionLatencyMillis = if (hasLiveSolution) intent.getDoubleExtra(
                    LoggingService.EXTRA_LIVE_SOLUTION_LATENCY_MS,
                    Double.NaN,
                ) else null,
                liveFgoSolutionAgeNanos = if (hasFgoSolution) intent.getLongExtra(
                    LoggingService.EXTRA_LIVE_FGO_SOLUTION_AGE_NANOS,
                    0L,
                ) else null,
            ))
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        screen = DashboardScreen(this, ::startRequested, ::stopRequested, ::runReplay, savedInstanceState)
        setContentView(screen.rootView)
        screen.setPermissionsGranted(hasRequiredPermissions())
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    override fun onResume() {
        super.onResume()
        if (!receiverRegistered) {
            val filter = IntentFilter(LoggingService.ACTION_STATUS)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                registerReceiver(statusReceiver, filter, RECEIVER_NOT_EXPORTED)
            } else {
                @Suppress("DEPRECATION")
                registerReceiver(statusReceiver, filter)
            }
            receiverRegistered = true
        }
        screen.setPermissionsGranted(hasRequiredPermissions())
        val logging = getSharedPreferences(LoggingService.PREFERENCES, MODE_PRIVATE)
            .getBoolean(LoggingService.KEY_LOGGING, false)
        if (logging) {
            startService(Intent(this, LoggingService::class.java).setAction(LoggingService.ACTION_QUERY))
        }
    }

    override fun onSaveInstanceState(outState: Bundle) { screen.save(outState); super.onSaveInstanceState(outState) }

    override fun onStart() { super.onStart(); screen.onStart() }

    override fun onStop() { screen.onStop(); super.onStop() }

    override fun onLowMemory() { super.onLowMemory(); screen.onLowMemory() }

    override fun onPause() {
        if (receiverRegistered) {
            unregisterReceiver(statusReceiver)
            receiverRegistered = false
        }
        super.onPause()
    }

    override fun onDestroy() {
        screen.onDestroy()
        replayExecutor.shutdownNow()
        super.onDestroy()
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != PERMISSION_REQUEST) return
        val granted = hasRequiredPermissions()
        screen.setPermissionsGranted(granted)
        if (granted && startAfterPermission) startServiceNow()
        startAfterPermission = false
    }

    private fun startRequested() {
        if (!hasRequiredPermissions()) {
            startAfterPermission = true
            requestPermissions(requiredPermissions().toTypedArray(), PERMISSION_REQUEST)
            return
        }
        startServiceNow()
    }

    private fun startServiceNow() {
        startForegroundService(Intent(this, LoggingService::class.java).setAction(LoggingService.ACTION_START))
    }

    private fun stopRequested() {
        startService(Intent(this, LoggingService::class.java).setAction(LoggingService.ACTION_STOP))
    }

    private fun runReplay() {
        if (replayRunning) return
        replayRunning = true
        screen.showReplayLoading()
        replayExecutor.execute {
            val outcome = runCatching { AssetReplayEstimator(applicationContext).run() }
            runOnUiThread {
                if (isDestroyed) return@runOnUiThread
                replayRunning = false
                outcome.onSuccess(screen::showReplayResult)
                    .onFailure { screen.showReplayFailure(it.message ?: it.javaClass.simpleName) }
            }
        }
    }

    private fun hasRequiredPermissions(): Boolean = requiredPermissions().all {
        checkSelfPermission(it) == PackageManager.PERMISSION_GRANTED
    }

    private fun requiredPermissions(): List<String> = buildList {
        add(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            add(Manifest.permission.POST_NOTIFICATIONS)
        }
    }

    companion object {
        private const val PERMISSION_REQUEST = 4102
    }
}
