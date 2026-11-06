package com.example.navgraph.live

import com.example.navgraph.estimator.Coordinates
import com.example.navgraph.estimator.EnuFrame
import com.example.navgraph.estimator.FgoConfig
import com.example.navgraph.estimator.GnssEpoch
import com.example.navgraph.estimator.GnssInsEkf
import com.example.navgraph.estimator.GnssWlsSolver
import com.example.navgraph.estimator.ImuSample
import com.example.navgraph.estimator.InertialFixedLagSmoother
import com.example.navgraph.estimator.InsState
import com.example.navgraph.estimator.NativeGnssWls
import com.example.navgraph.estimator.NativeGnssInsEkf
import com.example.navgraph.estimator.NativeFixedLagSmoother
import com.example.navgraph.estimator.NativeInsMechanization
import com.example.navgraph.estimator.NativeInsStatus
import com.example.navgraph.estimator.PositionEstimate
import com.example.navgraph.estimator.UnitQuaternion
import java.util.PriorityQueue
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sqrt

enum class LiveSolutionSource {
    WLS,
    EKF,
    FGO,
}

data class LiveSolutionSnapshot(
    val source: LiveSolutionSource,
    val elapsedRealtimeNanos: Long,
    val gnssTimestampMillis: Long,
    val latitudeDegrees: Double,
    val longitudeDegrees: Double,
    val altitudeMeters: Double,
    val horizontalSigmaMeters: Double,
    val verticalSigmaMeters: Double,
    val speedMetersPerSecond: Double,
    val solverLatencyMillis: Double,
    val converged: Boolean,
)

data class LiveBackendHealth(
    val state: String = "waiting_for_clean_epoch",
    val pairedImuSamples: Long = 0,
    val unpairedImuSamples: Long = 0,
    val maximumImuPairSkewMillis: Double = 0.0,
    val wlsUpdates: Long = 0,
    val wlsFailures: Long = 0,
    val wlsEngine: String = "KOTLIN",
    val nativeWlsFailures: Long = 0,
    val insEngine: String = "KOTLIN",
    val nativeInsPropagationUpdates: Long = 0,
    val nativeInsFailures: Long = 0,
    val ekfEngine: String = "KOTLIN",
    val nativeEkfPropagationUpdates: Long = 0,
    val nativeEkfGnssUpdates: Long = 0,
    val nativeEkfFailures: Long = 0,
    val ekfPropagationUpdates: Long = 0,
    val ekfGnssUpdates: Long = 0,
    val staleGnssUpdates: Long = 0,
    val fgoUpdates: Long = 0,
    val fgoFailures: Long = 0,
    val fgoEngine: String = "KOTLIN",
    val nativeFgoWindowUpdates: Long = 0,
    val nativeFgoFailures: Long = 0,
    val fgoSkippedBusy: Long = 0,
    val fgoWindowSolves: Long = 0,
    val fgoConvergedWindows: Long = 0,
    val fgoMaxWindowKeyframes: Int = 0,
    val fgoFeedbackAccepted: Long = 0,
    val fgoFeedbackRejected: Long = 0,
    val latestFgoFeedbackReason: String = "none",
    val latestFgoFeedbackCorrectionMeters: Double = 0.0,
    val resetCount: Long = 0,
    val latestSolution: LiveSolutionSnapshot? = null,
    val latestFgoSolution: LiveSolutionSnapshot? = null,
)

/**
 * Low-latency Kotlin reference backend. It is deliberately independent of
 * Android framework types so its ordering and reset behavior can be unit tested.
 */
class KotlinLiveEstimatorBackend(
    private val reorderDelayNanos: Long = 100_000_000L,
    private val smootherExecutor: ExecutorService = Executors.newSingleThreadExecutor(),
    private val applyEarthRotation: Boolean = true,
) : LiveEstimatorBackend, LiveEstimatorBackendHealthSource {
    private val lock = Any()
    private val wlsSolver = GnssWlsSolver(applyEarthRotation)
    private val assembler = ImuSampleAssembler()
    private val pendingImu = PriorityQueue<ImuSample>(compareBy { it.timestampNanos })
    private val pendingGnss = PriorityQueue<PendingGnss>(compareBy { it.epoch.integrationTimestampNanos })
    private val imuHistory = ArrayDeque<ImuSample>()
    private val epochHistory = ArrayDeque<GnssEpoch>()
    private val wlsHistory = ArrayDeque<PositionEstimate>()
    private val fgoRunning = AtomicBoolean(false)

    private var maximumSeenElapsedNanos = Long.MIN_VALUE
    private var previousImu: ImuSample? = null
    private var lastWls: PositionEstimate? = null
    private var frame: EnuFrame? = null
    private var ekf: GnssInsEkf? = null
    private var generation = 0L
    private var closed = false
    private var mutableHealth = LiveBackendHealth(
        wlsEngine = NativeGnssWls.readinessLabel,
        insEngine = NativeInsMechanization.readinessLabel,
        ekfEngine = NativeGnssInsEkf.readinessLabel,
        fgoEngine = NativeFixedLagSmoother.readinessLabel,
    )

    init {
        require(reorderDelayNanos >= 0L)
    }

    override fun reset(clockContinuityId: Int) = synchronized(lock) {
        generation++
        pendingImu.clear()
        pendingGnss.clear()
        imuHistory.clear()
        epochHistory.clear()
        wlsHistory.clear()
        assembler.reset()
        previousImu = null
        lastWls = null
        frame = null
        ekf = null
        maximumSeenElapsedNanos = Long.MIN_VALUE
        mutableHealth = mutableHealth.copy(
            state = "reset_clock_arc_$clockContinuityId",
            resetCount = mutableHealth.resetCount + 1,
            latestSolution = null,
            latestFgoSolution = null,
        )
    }

    override fun pushImu(sample: RawImuSample) = synchronized(lock) {
        if (closed) return@synchronized
        maximumSeenElapsedNanos = max(maximumSeenElapsedNanos, sample.elapsedRealtimeNanos)
        assembler.push(sample).forEach { combined ->
            pendingImu += combined
            insertImuHistory(combined)
        }
        updateAssemblerHealth()
        drain(watermark())
    }

    override fun pushGnssEpoch(epoch: GnssEpoch) = synchronized(lock) {
        if (closed) return@synchronized
        maximumSeenElapsedNanos = max(maximumSeenElapsedNanos, epoch.integrationTimestampNanos)
        val native = runCatching {
            NativeGnssWls.solveEpoch(epoch, lastWls, applyEarthRotation)
        }.getOrNull()
        val nativeUsable = native?.numericallyValid == true
        val estimate = if (nativeUsable) native.estimate else wlsSolver.solveEpoch(epoch, lastWls)
        if (NativeGnssWls.isAvailable && !nativeUsable) {
            mutableHealth = mutableHealth.copy(
                nativeWlsFailures = mutableHealth.nativeWlsFailures + 1,
            )
        }
        val plausible = estimate.converged && estimate.positionEcefMeters.all(Double::isFinite) &&
            norm(estimate.positionEcefMeters) in 5.0e6..8.0e6
        mutableHealth = if (plausible) {
            lastWls = estimate
            mutableHealth.copy(
                state = if (ekf == null) "wls_ready_waiting_for_imu" else mutableHealth.state,
                wlsUpdates = mutableHealth.wlsUpdates + 1,
                wlsEngine = if (nativeUsable) "C++ JNI" else "KOTLIN",
                latestSolution = if (ekf == null) snapshot(
                    estimate,
                    epoch.integrationTimestampNanos,
                    LiveSolutionSource.WLS,
                    horizontalSigmaMeters = Double.NaN,
                    verticalSigmaMeters = Double.NaN,
                ) else mutableHealth.latestSolution,
            )
        } else {
            mutableHealth.copy(
                state = "wls_failed",
                wlsFailures = mutableHealth.wlsFailures + 1,
            )
        }
        if (plausible) {
            pendingGnss += PendingGnss(epoch, estimate)
            epochHistory += epoch
            wlsHistory += estimate
            trimHistories(epoch.integrationTimestampNanos)
        }
        drain(watermark())
    }

    override fun updateFusedReference(sample: FusedLocationSample) = Unit

    override fun backendHealth(): LiveBackendHealth = synchronized(lock) { mutableHealth.copy() }

    override fun close() {
        synchronized(lock) {
            if (closed) return
            drain(Long.MAX_VALUE)
            closed = true
        }
        smootherExecutor.shutdownNow()
    }

    private fun watermark(): Long = if (maximumSeenElapsedNanos == Long.MIN_VALUE) {
        Long.MIN_VALUE
    } else {
        maximumSeenElapsedNanos - reorderDelayNanos
    }

    private fun drain(watermarkNanos: Long) {
        while (pendingImu.peek()?.timestampNanos?.let { it <= watermarkNanos } == true) {
            processImu(pendingImu.remove())
        }
        val last = previousImu
        while (last != null && pendingGnss.peek()?.epoch?.integrationTimestampNanos?.let {
                it <= minOf(last.timestampNanos, watermarkNanos)
            } == true
        ) {
            processLateGnss(pendingGnss.remove(), last)
        }
    }

    private fun processImu(current: ImuSample) {
        var intervalStart: ImuSample = previousImu ?: current.also { previousImu = it }
        while (pendingGnss.peek()?.epoch?.integrationTimestampNanos?.let { it <= current.timestampNanos } == true) {
            val pending = pendingGnss.remove()
            val epochTime = pending.epoch.integrationTimestampNanos
            if (epochTime < intervalStart.timestampNanos) {
                processLateGnss(pending, intervalStart)?.let { intervalStart = it }
                continue
            }
            val aligned = if (current.timestampNanos == intervalStart.timestampNanos) {
                current.copy(timestampNanos = epochTime)
            } else {
                interpolate(intervalStart, current, epochTime)
            }
            propagate(intervalStart, aligned)
            processGnss(pending, aligned)
            intervalStart = aligned
        }
        propagate(intervalStart, current)
        previousImu = current
        publishEkf(current.timestampNanos, lastWls?.timestampMillis ?: 0L, 0.0)
    }

    private fun processLateGnss(pending: PendingGnss, latestImu: ImuSample): ImuSample? {
        val age = latestImu.timestampNanos - pending.epoch.integrationTimestampNanos
        if (age > MAX_LATE_GNSS_NANOS) {
            mutableHealth = mutableHealth.copy(
                state = "stale_gnss_epoch",
                staleGnssUpdates = mutableHealth.staleGnssUpdates + 1,
            )
            return null
        }
        val aligned = latestImu.copy(timestampNanos = pending.epoch.integrationTimestampNanos)
        return if (ekf == null) {
            processGnss(pending, aligned)
            aligned
        } else {
            mutableHealth = mutableHealth.copy(
                state = "stale_gnss_epoch",
                staleGnssUpdates = mutableHealth.staleGnssUpdates + 1,
            )
            null
        }
    }

    private fun processGnss(pending: PendingGnss, alignedImu: ImuSample) {
        if (ekf == null) initializeEkf(pending, alignedImu)
        val active = ekf ?: return
        if (active.state.timestampNanos != pending.epoch.integrationTimestampNanos) {
            mutableHealth = mutableHealth.copy(
                state = "gnss_alignment_rejected",
                staleGnssUpdates = mutableHealth.staleGnssUpdates + 1,
            )
            return
        }
        val started = System.nanoTime()
        val nativeUpdatesBefore = active.health.nativeGnssUpdates
        val nativeFailuresBefore = active.health.nativeFailures
        active.update(pending.epoch)
        mutableHealth = mutableHealth.copy(
            state = "ekf_gnss_corrected",
            ekfGnssUpdates = mutableHealth.ekfGnssUpdates + 1,
            ekfEngine = active.engine,
            nativeEkfGnssUpdates = mutableHealth.nativeEkfGnssUpdates +
                (active.health.nativeGnssUpdates - nativeUpdatesBefore),
            nativeEkfFailures = mutableHealth.nativeEkfFailures +
                (active.health.nativeFailures - nativeFailuresBefore),
        )
        publishEkf(
            pending.epoch.integrationTimestampNanos,
            pending.epoch.timestampMillis,
            (System.nanoTime() - started) / 1.0e6,
        )
        scheduleFgo(pending.epoch.integrationTimestampNanos)
    }

    private fun initializeEkf(pending: PendingGnss, alignedImu: ImuSample) {
        val localFrame = EnuFrame(pending.wls.positionEcefMeters)
        frame = localFrame
        ekf = GnssInsEkf(
            frame = localFrame,
            initialState = InsState(
                timestampNanos = pending.epoch.integrationTimestampNanos,
                positionEnuMeters = localFrame.ecefPositionToEnu(pending.wls.positionEcefMeters),
                velocityEnuMetersPerSecond = localFrame.ecefVectorToEnu(
                    pending.wls.velocityEcefMetersPerSecond,
                ),
                attitudeBodyToEnu = gravityAlignedAttitude(
                    alignedImu.specificForceBodyMetersPerSecondSquared,
                ),
                accelerometerBiasBodyMetersPerSecondSquared = DoubleArray(3),
                gyroBiasBodyRadiansPerSecond = DoubleArray(3),
                clockBiasMeters = pending.wls.clockBiasMeters,
                clockDriftMetersPerSecond = pending.wls.clockDriftMetersPerSecond,
            ),
            applyEarthRotation = applyEarthRotation,
            nominalPropagator = ::propagateNominal,
            preferNative = true,
        )
        mutableHealth = mutableHealth.copy(state = "ekf_initialized_gravity_aligned")
    }

    private fun propagate(previous: ImuSample, current: ImuSample) {
        val active = ekf ?: return
        if (current.timestampNanos <= previous.timestampNanos) return
        if (active.state.timestampNanos != previous.timestampNanos) return
        val nativeStepsBefore = active.health.nativePropagationSteps
        val nativeFailuresBefore = active.health.nativeFailures
        active.propagate(previous, current)
        val nativeSteps = active.health.nativePropagationSteps - nativeStepsBefore
        mutableHealth = mutableHealth.copy(
            state = "ekf_propagating",
            ekfPropagationUpdates = mutableHealth.ekfPropagationUpdates + 1,
            insEngine = if (nativeSteps > 0) "C++ JNI" else mutableHealth.insEngine,
            nativeInsPropagationUpdates = mutableHealth.nativeInsPropagationUpdates + nativeSteps,
            ekfEngine = active.engine,
            nativeEkfPropagationUpdates = mutableHealth.nativeEkfPropagationUpdates + nativeSteps,
            nativeEkfFailures = mutableHealth.nativeEkfFailures +
                (active.health.nativeFailures - nativeFailuresBefore),
        )
    }

    private fun propagateNominal(
        state: InsState,
        previous: ImuSample,
        current: ImuSample,
    ): InsState {
        val native = runCatching {
            NativeInsMechanization.propagate(state, previous, current)
        }.getOrNull()
        if (native?.status == NativeInsStatus.OK &&
            native.state.timestampNanos == current.timestampNanos &&
            native.state.numericallyValid()
        ) {
            mutableHealth = mutableHealth.copy(
                insEngine = "C++ JNI",
                nativeInsPropagationUpdates = mutableHealth.nativeInsPropagationUpdates + 1,
            )
            return native.state
        }
        if (NativeInsMechanization.isAvailable) {
            mutableHealth = mutableHealth.copy(
                insEngine = "KOTLIN FALLBACK",
                nativeInsFailures = mutableHealth.nativeInsFailures + 1,
            )
        }
        return com.example.navgraph.estimator.ImuMechanization.propagate(
            state,
            previous,
            current,
        )
    }

    private fun publishEkf(elapsedNanos: Long, gnssTimestampMillis: Long, latencyMillis: Double) {
        val active = ekf ?: return
        val localFrame = frame ?: return
        val position = localFrame.enuPositionToEcef(active.state.positionEnuMeters)
        val velocity = localFrame.enuVectorToEcef(active.state.velocityEnuMetersPerSecond)
        val sigma = active.positionStandardDeviationMeters()
        mutableHealth = mutableHealth.copy(
            latestSolution = snapshot(
                PositionEstimate(
                    timestampMillis = gnssTimestampMillis,
                    positionEcefMeters = position,
                    velocityEcefMetersPerSecond = velocity,
                    clockBiasMeters = active.state.clockBiasMeters,
                    clockDriftMetersPerSecond = active.state.clockDriftMetersPerSecond,
                    iterations = 1,
                    observationCount = 0,
                    rejectedCount = active.health.rejectedMeasurements,
                    latencyMillis = latencyMillis,
                    converged = sigma.all(Double::isFinite),
                ),
                elapsedNanos,
                LiveSolutionSource.EKF,
                horizontalSigmaMeters = hypot(sigma[0], sigma[1]),
                verticalSigmaMeters = sigma[2],
            ),
        )
    }

    private fun scheduleFgo(endNanos: Long) {
        val localFrame = frame ?: return
        val epochs = epochHistory.filter { it.integrationTimestampNanos <= endNanos &&
            endNanos - it.integrationTimestampNanos <= HISTORY_NANOS }
        if (epochs.size < 2) return
        val firstNanos = epochs.first().integrationTimestampNanos
        val samples = imuHistory.filter { it.timestampNanos in firstNanos..endNanos }
        val estimatesByTimestamp = wlsHistory.associateBy { it.timestampMillis }
        val wls = epochs.mapNotNull { estimatesByTimestamp[it.timestampMillis] }
        if (wls.size != epochs.size || samples.size < 2) return
        val activeEkf = ekf ?: return
        if (activeEkf.state.timestampNanos != endNanos) return
        val feedbackReference = FgoFeedbackReference(
            elapsedRealtimeNanos = endNanos,
            state = activeEkf.state.deepCopy(),
            variance = activeEkf.translationalVariance(),
        )
        if (!fgoRunning.compareAndSet(false, true)) {
            mutableHealth = mutableHealth.copy(fgoSkippedBusy = mutableHealth.fgoSkippedBusy + 1)
            return
        }
        val taskGeneration = generation
        smootherExecutor.execute {
            val started = System.nanoTime()
            val result = runCatching {
                InertialFixedLagSmoother(
                    localFrame,
                    lagSeconds = FgoConfig.DEFAULT_LAG_SECONDS,
                    maxKeyframes = FgoConfig.MAX_KEYFRAMES,
                    applyEarthRotation = applyEarthRotation,
                    preferNative = true,
                ).run(
                    epochs,
                    samples,
                    wls,
                    latestOnly = true,
                )
            }
            synchronized(lock) {
                if (taskGeneration == generation && !closed) {
                    result.onSuccess { smoothed ->
                        val estimate = smoothed.estimates.last()
                        val uncertainty = smoothed.stateUncertainties.lastOrNull()
                        val currentEkf = ekf
                        val decision = if (currentEkf == null) {
                            FgoFeedbackDecision(false, "ekf_unavailable")
                        } else {
                            FgoFeedbackGate.evaluate(
                                feedbackReference,
                                estimate,
                                uncertainty,
                                localFrame,
                                currentEkf.state.timestampNanos,
                            )
                        }
                        if (decision.accepted && currentEkf != null) {
                            currentEkf.applyCorrelatedSmootherDelta(
                                decision.positionDeltaEnuMeters,
                                decision.velocityDeltaEnuMetersPerSecond,
                                decision.clockBiasDeltaMeters,
                                decision.clockDriftDeltaMetersPerSecond,
                                FgoFeedbackGate.FEEDBACK_GAIN,
                            )
                        }
                        mutableHealth = mutableHealth.copy(
                            state = if (decision.accepted) {
                                "fgo_feedback_applied"
                            } else {
                                "fgo_feedback_rejected_${decision.reason}"
                            },
                            fgoUpdates = mutableHealth.fgoUpdates + 1,
                            fgoEngine = smoothed.engine,
                            nativeFgoWindowUpdates = mutableHealth.nativeFgoWindowUpdates +
                                smoothed.nativeWindowUpdates,
                            nativeFgoFailures = mutableHealth.nativeFgoFailures +
                                smoothed.nativeFailures,
                            fgoWindowSolves = mutableHealth.fgoWindowSolves +
                                smoothed.windowSolves,
                            fgoConvergedWindows = mutableHealth.fgoConvergedWindows +
                                smoothed.convergedWindows,
                            fgoMaxWindowKeyframes = maxOf(
                                mutableHealth.fgoMaxWindowKeyframes,
                                smoothed.maxWindowKeyframes,
                            ),
                            fgoFeedbackAccepted = mutableHealth.fgoFeedbackAccepted +
                                if (decision.accepted) 1 else 0,
                            fgoFeedbackRejected = mutableHealth.fgoFeedbackRejected +
                                if (decision.accepted) 0 else 1,
                            latestFgoFeedbackReason = decision.reason,
                            latestFgoFeedbackCorrectionMeters =
                                decision.positionCorrectionNormMeters * FgoFeedbackGate.FEEDBACK_GAIN,
                            latestFgoSolution = snapshot(
                                estimate.copy(latencyMillis = (System.nanoTime() - started) / 1.0e6),
                                epochs.last().integrationTimestampNanos,
                                LiveSolutionSource.FGO,
                                horizontalSigmaMeters = uncertainty?.positionSigmaEnuMeters?.let {
                                    hypot(it[0], it[1])
                                } ?: Double.NaN,
                                verticalSigmaMeters = uncertainty?.positionSigmaEnuMeters?.get(2)
                                    ?: Double.NaN,
                            ),
                        )
                        if (decision.accepted && currentEkf != null) {
                            publishEkf(
                                currentEkf.state.timestampNanos,
                                lastWls?.timestampMillis ?: 0L,
                                (System.nanoTime() - started) / 1.0e6,
                            )
                        }
                    }.onFailure {
                        mutableHealth = mutableHealth.copy(
                            fgoFailures = mutableHealth.fgoFailures + 1,
                            state = "fgo_failed_${it.javaClass.simpleName}",
                        )
                    }
                }
                fgoRunning.set(false)
            }
        }
    }

    private fun trimHistories(latestNanos: Long) {
        val cutoff = latestNanos - HISTORY_NANOS
        while (epochHistory.firstOrNull()?.integrationTimestampNanos?.let { it < cutoff } == true) {
            epochHistory.removeFirst()
            wlsHistory.removeFirst()
        }
        while (imuHistory.size > 2 && imuHistory[1].timestampNanos < cutoff) {
            imuHistory.removeFirst()
        }
    }

    private fun insertImuHistory(sample: ImuSample) {
        if (imuHistory.isEmpty() || sample.timestampNanos > imuHistory.last().timestampNanos) {
            imuHistory += sample
        }
    }

    private fun updateAssemblerHealth() {
        mutableHealth = mutableHealth.copy(
            pairedImuSamples = assembler.pairedSamples,
            unpairedImuSamples = assembler.discardedSamples,
            maximumImuPairSkewMillis = assembler.maximumPairSkewNanos / 1.0e6,
        )
    }

    private fun interpolate(first: ImuSample, second: ImuSample, timestampNanos: Long): ImuSample {
        require(timestampNanos in first.timestampNanos..second.timestampNanos)
        if (timestampNanos == first.timestampNanos) return first
        if (timestampNanos == second.timestampNanos) return second
        val fraction = (timestampNanos - first.timestampNanos).toDouble() /
            (second.timestampNanos - first.timestampNanos).toDouble()
        return ImuSample(
            timestampNanos,
            DoubleArray(3) { first.specificForceBodyMetersPerSecondSquared[it] +
                fraction * (second.specificForceBodyMetersPerSecondSquared[it] -
                    first.specificForceBodyMetersPerSecondSquared[it]) },
            DoubleArray(3) { first.angularRateBodyRadiansPerSecond[it] +
                fraction * (second.angularRateBodyRadiansPerSecond[it] -
                    first.angularRateBodyRadiansPerSecond[it]) },
        )
    }

    private fun gravityAlignedAttitude(specificForceBody: DoubleArray): UnitQuaternion {
        val magnitude = norm(specificForceBody)
        if (!magnitude.isFinite() || magnitude < 1.0) return UnitQuaternion.IDENTITY
        val from = DoubleArray(3) { specificForceBody[it] / magnitude }
        val dot = from[2].coerceIn(-1.0, 1.0)
        if (dot > 1.0 - 1.0e-10) return UnitQuaternion.IDENTITY
        if (dot < -1.0 + 1.0e-10) return UnitQuaternion.fromRotationVector(
            doubleArrayOf(Math.PI, 0.0, 0.0),
        )
        val axis = doubleArrayOf(from[1], -from[0], 0.0)
        val axisNorm = norm(axis)
        val angle = acos(dot)
        return UnitQuaternion.fromRotationVector(DoubleArray(3) { axis[it] * angle / axisNorm })
    }

    private fun snapshot(
        estimate: PositionEstimate,
        elapsedNanos: Long,
        source: LiveSolutionSource,
        horizontalSigmaMeters: Double,
        verticalSigmaMeters: Double,
    ): LiveSolutionSnapshot {
        val lla = Coordinates.ecefToLla(estimate.positionEcefMeters)
        return LiveSolutionSnapshot(
            source,
            elapsedNanos,
            estimate.timestampMillis,
            lla[0],
            lla[1],
            lla[2],
            horizontalSigmaMeters,
            verticalSigmaMeters,
            norm(estimate.velocityEcefMetersPerSecond),
            estimate.latencyMillis,
            estimate.converged,
        )
    }

    private fun norm(values: DoubleArray): Double = sqrt(values.sumOf { it * it })

    private fun InsState.deepCopy(): InsState = copy(
        positionEnuMeters = positionEnuMeters.copyOf(),
        velocityEnuMetersPerSecond = velocityEnuMetersPerSecond.copyOf(),
        accelerometerBiasBodyMetersPerSecondSquared =
            accelerometerBiasBodyMetersPerSecondSquared.copyOf(),
        gyroBiasBodyRadiansPerSecond = gyroBiasBodyRadiansPerSecond.copyOf(),
    )

    private fun InsState.numericallyValid(): Boolean =
        timestampNanos != Long.MIN_VALUE &&
            positionEnuMeters.size == 3 && positionEnuMeters.all(Double::isFinite) &&
            velocityEnuMetersPerSecond.size == 3 &&
            velocityEnuMetersPerSecond.all(Double::isFinite) &&
            accelerometerBiasBodyMetersPerSecondSquared.size == 3 &&
            accelerometerBiasBodyMetersPerSecondSquared.all(Double::isFinite) &&
            gyroBiasBodyRadiansPerSecond.size == 3 &&
            gyroBiasBodyRadiansPerSecond.all(Double::isFinite) &&
            listOf(
                attitudeBodyToEnu.w,
                attitudeBodyToEnu.x,
                attitudeBodyToEnu.y,
                attitudeBodyToEnu.z,
                clockBiasMeters,
                clockDriftMetersPerSecond,
            ).all(Double::isFinite)

    private data class PendingGnss(val epoch: GnssEpoch, val wls: PositionEstimate)

    private companion object {
        const val MAX_LATE_GNSS_NANOS = 100_000_000L
        // Retain the FGO horizon plus one epoch so the live worker can always
        // build a full fixed-lag window without reusing stale history.
        val HISTORY_NANOS =
            ((FgoConfig.DEFAULT_LAG_SECONDS + 1.0) * 1_000_000_000.0).toLong()
    }
}

internal class ImuSampleAssembler(
    private val maximumSkewNanos: Long = 20_000_000L,
) {
    private val accelerometer = mutableListOf<RawImuSample>()
    private val gyroscope = mutableListOf<RawImuSample>()
    var pairedSamples = 0L
        private set
    var discardedSamples = 0L
        private set
    var maximumPairSkewNanos = 0L
        private set

    init {
        require(maximumSkewNanos >= 0L)
    }

    fun push(sample: RawImuSample): List<ImuSample> {
        val target = when (sample.sensorType) {
            ImuSensorType.ACCELEROMETER -> accelerometer
            ImuSensorType.GYROSCOPE -> gyroscope
        }
        target += sample
        target.sortBy { it.elapsedRealtimeNanos }
        val output = mutableListOf<ImuSample>()
        while (accelerometer.isNotEmpty() && gyroscope.isNotEmpty()) {
            val accel = accelerometer.first()
            val gyroIndex = gyroscope.indices.minByOrNull {
                abs(gyroscope[it].elapsedRealtimeNanos - accel.elapsedRealtimeNanos)
            } ?: break
            val gyro = gyroscope[gyroIndex]
            val skew = abs(gyro.elapsedRealtimeNanos - accel.elapsedRealtimeNanos)
            if (skew <= maximumSkewNanos) {
                accelerometer.removeAt(0)
                gyroscope.removeAt(gyroIndex)
                maximumPairSkewNanos = max(maximumPairSkewNanos, skew)
                pairedSamples++
                output += ImuSample(
                    accel.elapsedRealtimeNanos,
                    accel.values.copyOf(),
                    gyro.values.copyOf(),
                )
            } else if (gyro.elapsedRealtimeNanos < accel.elapsedRealtimeNanos) {
                gyroscope.removeAt(gyroIndex)
                discardedSamples++
            } else {
                accelerometer.removeAt(0)
                discardedSamples++
            }
        }
        while (accelerometer.size > MAX_PENDING_SAMPLES) {
            accelerometer.removeAt(0)
            discardedSamples++
        }
        while (gyroscope.size > MAX_PENDING_SAMPLES) {
            gyroscope.removeAt(0)
            discardedSamples++
        }
        return output
    }

    fun reset() {
        discardedSamples += accelerometer.size + gyroscope.size
        accelerometer.clear()
        gyroscope.clear()
    }

    private companion object {
        const val MAX_PENDING_SAMPLES = 32
    }
}
