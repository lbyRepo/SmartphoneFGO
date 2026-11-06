package com.example.navgraph.estimator

import java.io.InputStream
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

class GsdcReplayEstimator private constructor(
    private val deviceRows: List<Map<String, String>>,
    private val truthRows: List<Map<String, String>>,
    private val lagSeconds: Double = 2.0,
    private val datasetName: String = "GSDC replay",
    private val datasetScope: String = "Public derived GNSS replay",
) : ReplayEstimator {
    override fun run(): ReplayResult {
        val legacySchema = deviceRows.firstOrNull()?.containsKey("millisSinceGpsEpoch") == true
        val (epochs, preprocessingRejections) = preprocess(deviceRows, legacySchema)
        require(epochs.isNotEmpty()) { "No usable GNSS epochs after preprocessing" }
        val truth = loadTruth(truthRows)
        val wls = runWls(epochs)
        val fgo = runFixedLagFgo(epochs, wls)
        val jacobians = checkJacobians(wls.first(), epochs.first().observations.first())
        require(jacobians.first < 2.0e-5 &&
            (!jacobians.second.isFinite() || jacobians.second < 2.0e-5)
        ) {
            "Numerical Jacobian check failed: pseudorange=${jacobians.first}, doppler=${jacobians.second}"
        }
        val reference = truth.values.first()
        return ReplayResult(
            datasetName = datasetName,
            epochCount = epochs.size,
            acceptedObservationCount = epochs.sumOf { it.observations.size },
            preprocessingRejections = preprocessingRejections,
            wlsMetrics = computeMetrics(wls, truth),
            fgoMetrics = computeMetrics(fgo, truth),
            geographicTracks = wls.map { com.example.navgraph.map.geographicFromEcef(
                "WLS", it.timestampMillis, it.positionEcefMeters, it.converged) } +
                fgo.map { com.example.navgraph.map.geographicFromEcef(
                    "FGO", it.timestampMillis, it.positionEcefMeters, it.converged) } +
                truth.map { (time, position) -> com.example.navgraph.map.geographicFromEcef(
                    "Reference", time, position) },
            truthTrajectory = truth.values.map { it.toHorizontal(reference) },
            wlsTrajectory = wls.map { it.positionEcefMeters.toHorizontal(reference) },
            fgoTrajectory = fgo.map { it.positionEcefMeters.toHorizontal(reference) },
            pseudorangeJacobianError = jacobians.first,
            dopplerJacobianError = jacobians.second,
            datasetScope = datasetScope,
            measurementProfile = if (legacySchema) {
                "Pseudorange only; Doppler / C/N0 / elevation / IMU unavailable"
            } else {
                "Derived pseudorange + Doppler with frequency, state, C/N0, and elevation QC"
            },
        )
    }

    private fun preprocess(
        rows: List<Map<String, String>>,
        legacySchema: Boolean,
    ): Pair<List<GnssEpoch>, Map<String, Int>> {
        val grouped = sortedMapOf<Long, MutableList<GnssObservation>>()
        val rejections = linkedMapOf<String, Int>()
        fun reject(reason: String) {
            rejections[reason] = rejections.getOrDefault(reason, 0) + 1
        }

        rows.forEach { row ->
            if (!legacySchema && row["MessageType"] != "Raw") {
                reject("not_raw")
                return@forEach
            }
            val timestamp = row.long(if (legacySchema) "millisSinceGpsEpoch" else "utcTimeMillis")
            val constellation = row.int(if (legacySchema) "constellationType" else "ConstellationType")
            val state = if (legacySchema) 1 else row.int("State")
            val carrierFrequency = if (legacySchema) null else row.double("CarrierFrequencyHz")
            val cn0 = if (legacySchema) null else row.double("Cn0DbHz")
            val elevation = if (legacySchema) null else row.double("SvElevationDegrees")
            if (timestamp == null || constellation == null || constellation !in SUPPORTED_CONSTELLATIONS) {
                reject("unsupported_constellation_or_time")
                return@forEach
            }
            if (state == null || state == 0) {
                reject("invalid_tracking_state")
                return@forEach
            }
            if (!legacySchema) {
                if (carrierFrequency == null || carrierFrequency !in 1.1e9..1.7e9) {
                    reject("unsupported_frequency")
                    return@forEach
                }
                if (cn0 == null || elevation == null || cn0 < 18.0 || elevation < 5.0) {
                    reject("low_signal_or_elevation")
                    return@forEach
                }
            }
            val rawPseudorange = row.double(if (legacySchema) "rawPrM" else "RawPseudorangeMeters")
            val pseudorangeSigma = row.double(if (legacySchema) "rawPrUncM" else "RawPseudorangeUncertaintyMeters")
            val satelliteClock = row.double(if (legacySchema) "satClkBiasM" else "SvClockBiasMeters")
            val interSignalBias = row.double(if (legacySchema) "isrbM" else "IsrbMeters") ?: 0.0
            val ionosphere = row.double(if (legacySchema) "ionoDelayM" else "IonosphericDelayMeters") ?: 0.0
            val troposphere = row.double(if (legacySchema) "tropoDelayM" else "TroposphericDelayMeters") ?: 0.0
            val satellitePosition = doubleArrayOf(
                row.double(if (legacySchema) "xSatPosM" else "SvPositionXEcefMeters") ?: Double.NaN,
                row.double(if (legacySchema) "ySatPosM" else "SvPositionYEcefMeters") ?: Double.NaN,
                row.double(if (legacySchema) "zSatPosM" else "SvPositionZEcefMeters") ?: Double.NaN,
            )
            if (rawPseudorange == null || pseudorangeSigma == null || satelliteClock == null ||
                !satellitePosition.all(Double::isFinite)
            ) {
                reject("missing_pseudorange_input")
                return@forEach
            }
            if (rawPseudorange !in 1.0e6..1.0e8 || pseudorangeSigma <= 0.0 ||
                pseudorangeSigma > 100.0 || norm(satellitePosition) < 2.0e7
            ) {
                reject("invalid_pseudorange")
                return@forEach
            }
            val rawRate = if (legacySchema) null else row.double("PseudorangeRateMetersPerSecond")
            val rangeRateSigma = if (legacySchema) null else
                row.double("PseudorangeRateUncertaintyMetersPerSecond")
            val satelliteClockDrift = row.double(
                if (legacySchema) "satClkDriftMps" else "SvClockDriftMetersPerSecond",
            ) ?: 0.0
            val satelliteVelocity = doubleArrayOf(
                row.double(if (legacySchema) "xSatVelMps" else "SvVelocityXEcefMetersPerSecond") ?: Double.NaN,
                row.double(if (legacySchema) "ySatVelMps" else "SvVelocityYEcefMetersPerSecond") ?: Double.NaN,
                row.double(if (legacySchema) "zSatVelMps" else "SvVelocityZEcefMetersPerSecond") ?: Double.NaN,
            )
            val rateValid = rawRate != null && rangeRateSigma != null &&
                rangeRateSigma in 0.0..20.0 && rangeRateSigma > 0.0 && satelliteVelocity.all(Double::isFinite)
            grouped.getOrPut(timestamp) { mutableListOf() } += GnssObservation(
                timestampMillis = timestamp,
                constellation = constellation,
                svid = row.int(if (legacySchema) "svid" else "Svid") ?: -1,
                signalType = row[if (legacySchema) "signalType" else "SignalType"]
                    .orEmpty().ifBlank { "UNKNOWN" },
                satellitePositionMeters = satellitePosition,
                satelliteVelocityMetersPerSecond = if (rateValid) satelliteVelocity else DoubleArray(3) { Double.NaN },
                correctedPseudorangeMeters = rawPseudorange + satelliteClock - interSignalBias - ionosphere - troposphere,
                pseudorangeSigmaMeters = max(pseudorangeSigma, 1.0),
                correctedRangeRateMetersPerSecond = if (rateValid) rawRate!! + satelliteClockDrift else Double.NaN,
                rangeRateSigmaMetersPerSecond = if (rateValid) max(rangeRateSigma!!, 0.15) else Double.NaN,
                cn0DbHz = cn0 ?: Double.NaN,
                elevationDegrees = elevation ?: Double.NaN,
            )
        }
        val epochs = grouped.map { (timestamp, observations) -> GnssEpoch(timestamp, observations) }
            .filter { it.observations.size >= 4 }
        return epochs to rejections
    }

    private fun loadTruth(rows: List<Map<String, String>>): LinkedHashMap<Long, DoubleArray> {
        val truth = linkedMapOf<Long, DoubleArray>()
        val legacySchema = rows.firstOrNull()?.containsKey("millisSinceGpsEpoch") == true
        rows.forEach { row ->
            if (!legacySchema && row["MessageType"] != "Fix") return@forEach
            val timestamp = row.long(if (legacySchema) "millisSinceGpsEpoch" else "UnixTimeMillis")
                ?: return@forEach
            val latitude = row.double(if (legacySchema) "latDeg" else "LatitudeDegrees")
                ?: return@forEach
            val longitude = row.double(if (legacySchema) "lngDeg" else "LongitudeDegrees")
                ?: return@forEach
            val altitude = row.double(
                if (legacySchema) "heightAboveWgs84EllipsoidM" else "AltitudeMeters",
            ) ?: return@forEach
            truth[timestamp] = Coordinates.llaToEcef(latitude, longitude, altitude)
        }
        require(truth.isNotEmpty()) { "Replay ground truth is empty" }
        return truth
    }

    private fun runWls(epochs: List<GnssEpoch>): List<PositionEstimate> {
        val estimates = mutableListOf<PositionEstimate>()
        val state = DoubleArray(4)
        epochs.forEach { epoch ->
            val started = System.nanoTime()
            var converged = false
            var iterations = 0
            for (iteration in 1..20) {
                iterations = iteration
                val equations = NormalEquations(4)
                epoch.observations.forEach { observation ->
                    val (residual, row) = pseudorangeWlsFactor(state, observation)
                    val standardized = residual / observation.pseudorangeSigmaMeters
                    val scale = sqrt(huberWeight(standardized)) / observation.pseudorangeSigmaMeters
                    equations.add(row, residual, scale)
                }
                val step = equations.solve(1.0e-9) ?: break
                for (index in state.indices) state[index] += step[index]
                if (norm(step.copyOfRange(0, 3)) < 1.0e-3 && abs(step[3]) < 1.0e-3) {
                    converged = true
                    break
                }
            }
            val rejected = epoch.observations.count { observation ->
                val (residual, _) = pseudorangeWlsFactor(state, observation)
                abs(residual / observation.pseudorangeSigmaMeters) > 8.0
            }
            estimates += PositionEstimate(
                timestampMillis = epoch.timestampMillis,
                positionEcefMeters = state.copyOfRange(0, 3),
                velocityEcefMetersPerSecond = DoubleArray(3),
                clockBiasMeters = state[3],
                clockDriftMetersPerSecond = 0.0,
                iterations = iterations,
                observationCount = epoch.observations.size,
                rejectedCount = rejected,
                latencyMillis = (System.nanoTime() - started) / 1.0e6,
                converged = converged,
            )
        }
        return estimates
    }

    private fun runFixedLagFgo(
        epochs: List<GnssEpoch>,
        wls: List<PositionEstimate>,
    ): List<PositionEstimate> = epochs.indices.map { endIndex ->
        var startIndex = endIndex
        while (startIndex > 0 &&
            epochs[endIndex].timestampMillis - epochs[startIndex - 1].timestampMillis <= lagSeconds * 1_000.0
        ) {
            startIndex--
        }
        val started = System.nanoTime()
        val windowEpochs = epochs.subList(startIndex, endIndex + 1)
        val windowWls = wls.subList(startIndex, endIndex + 1)
        val (states, iterations, converged) = solveWindow(windowEpochs, windowWls)
        val state = states.last()
        val rejected = epochs[endIndex].observations.count { observation ->
            val (residual, _) = pseudorangeFactor(state, observation)
            abs(residual / observation.pseudorangeSigmaMeters) > 8.0
        }
        PositionEstimate(
            timestampMillis = epochs[endIndex].timestampMillis,
            positionEcefMeters = state.copyOfRange(0, 3),
            velocityEcefMetersPerSecond = state.copyOfRange(3, 6),
            clockBiasMeters = state[6],
            clockDriftMetersPerSecond = state[7],
            iterations = iterations,
            observationCount = epochs[endIndex].observations.size,
            rejectedCount = rejected,
            latencyMillis = (System.nanoTime() - started) / 1.0e6,
            converged = converged,
        )
    }

    private fun solveWindow(
        epochs: List<GnssEpoch>,
        wls: List<PositionEstimate>,
    ): Triple<List<DoubleArray>, Int, Boolean> {
        val stateCount = epochs.size
        val dimension = stateCount * STATE_SIZE
        val states = Array(stateCount) { index -> DoubleArray(STATE_SIZE).apply {
            wls[index].positionEcefMeters.copyInto(this, 0)
            this[6] = wls[index].clockBiasMeters
        } }
        for (index in 1 until stateCount) {
            val dt = max((epochs[index].timestampMillis - epochs[index - 1].timestampMillis) / 1_000.0, 1.0e-3)
            for (axis in 0..2) {
                states[index - 1][axis + 3] = (states[index][axis] - states[index - 1][axis]) / dt
            }
            states[index - 1][7] = (states[index][6] - states[index - 1][6]) / dt
        }
        if (stateCount > 1) {
            for (index in 3..5) states.last()[index] = states[stateCount - 2][index]
            states.last()[7] = states[stateCount - 2][7]
        }

        var converged = false
        var iterations = 0
        for (iteration in 1..15) {
            iterations = iteration
            val equations = NormalEquations(dimension)
            fun addFactor(epochIndex: Int, residual: Double, localRow: DoubleArray, sigma: Double, robust: Boolean) {
                val row = DoubleArray(dimension)
                localRow.copyInto(row, epochIndex * STATE_SIZE)
                val standardized = residual / sigma
                val scale = sqrt(if (robust) huberWeight(standardized) else 1.0) / sigma
                equations.add(row, residual, scale)
            }

            val prior = DoubleArray(STATE_SIZE).apply {
                wls.first().positionEcefMeters.copyInto(this, 0)
                this[6] = wls.first().clockBiasMeters
            }
            val priorSigmas = doubleArrayOf(30.0, 30.0, 30.0, 20.0, 20.0, 20.0, 1_000.0, 100.0)
            for (component in 0 until STATE_SIZE) {
                val row = DoubleArray(STATE_SIZE).apply { this[component] = 1.0 }
                addFactor(0, states[0][component] - prior[component], row, priorSigmas[component], false)
            }

            epochs.forEachIndexed { epochIndex, epoch ->
                epoch.observations.forEach { observation ->
                    val (pseudorangeResidual, pseudorangeRow) = pseudorangeFactor(states[epochIndex], observation)
                    addFactor(epochIndex, pseudorangeResidual, pseudorangeRow,
                        observation.pseudorangeSigmaMeters, true)
                    if (observation.correctedRangeRateMetersPerSecond.isFinite() &&
                        observation.satelliteVelocityMetersPerSecond.all(Double::isFinite)
                    ) {
                        val (dopplerResidual, dopplerRow) = dopplerFactor(states[epochIndex], observation)
                        addFactor(epochIndex, dopplerResidual, dopplerRow,
                            observation.rangeRateSigmaMetersPerSecond, true)
                    }
                }
            }

            for (index in 1 until stateCount) {
                val dt = max((epochs[index].timestampMillis - epochs[index - 1].timestampMillis) / 1_000.0, 1.0e-3)
                val previous = (index - 1) * STATE_SIZE
                val current = index * STATE_SIZE
                for (axis in 0..2) {
                    val positionRow = DoubleArray(dimension).apply {
                        this[current + axis] = 1.0
                        this[previous + axis] = -1.0
                        this[previous + axis + 3] = -dt
                    }
                    val positionResidual = states[index][axis] - states[index - 1][axis] -
                        dt * states[index - 1][axis + 3]
                    equations.add(positionRow, positionResidual, 1.0 / 3.0)
                    val velocityRow = DoubleArray(dimension).apply {
                        this[current + axis + 3] = 1.0
                        this[previous + axis + 3] = -1.0
                    }
                    val velocityResidual = states[index][axis + 3] - states[index - 1][axis + 3]
                    equations.add(velocityRow, velocityResidual, 1.0 / 3.0)
                }
                val clockRow = DoubleArray(dimension).apply {
                    this[current + 6] = 1.0
                    this[previous + 6] = -1.0
                    this[previous + 7] = -dt
                }
                val clockResidual = states[index][6] - states[index - 1][6] - dt * states[index - 1][7]
                equations.add(clockRow, clockResidual, 1.0 / 100.0)
                val driftRow = DoubleArray(dimension).apply {
                    this[current + 7] = 1.0
                    this[previous + 7] = -1.0
                }
                equations.add(driftRow, states[index][7] - states[index - 1][7], 1.0 / 50.0)
            }

            val step = equations.solve(1.0e-6) ?: break
            var maxPositionStep = 0.0
            states.forEachIndexed { stateIndex, state ->
                val offset = stateIndex * STATE_SIZE
                for (component in 0 until STATE_SIZE) state[component] += step[offset + component]
                maxPositionStep = max(maxPositionStep, norm(step.copyOfRange(offset, offset + 3)))
            }
            if (maxPositionStep < 1.0e-3) {
                converged = true
                break
            }
        }
        return Triple(states.toList(), iterations, converged)
    }

    private fun pseudorangeWlsFactor(
        state: DoubleArray,
        observation: GnssObservation,
    ): Pair<Double, DoubleArray> {
        val transitSeconds = (observation.correctedPseudorangeMeters - state[3]) / SPEED_OF_LIGHT_MPS
        val satellite = rotateSatellite(observation.satellitePositionMeters, transitSeconds)
        val delta = subtract(state.copyOfRange(0, 3), satellite)
        val distance = norm(delta)
        val unit = DoubleArray(3) { delta[it] / distance }
        return distance + state[3] - observation.correctedPseudorangeMeters to
            doubleArrayOf(unit[0], unit[1], unit[2], 1.0)
    }

    private fun pseudorangeFactor(
        state: DoubleArray,
        observation: GnssObservation,
    ): Pair<Double, DoubleArray> {
        val transitSeconds = (observation.correctedPseudorangeMeters - state[6]) / SPEED_OF_LIGHT_MPS
        val satellite = rotateSatellite(observation.satellitePositionMeters, transitSeconds)
        val delta = subtract(state.copyOfRange(0, 3), satellite)
        val distance = norm(delta)
        val row = DoubleArray(STATE_SIZE)
        for (axis in 0..2) row[axis] = delta[axis] / distance
        row[6] = 1.0
        return distance + state[6] - observation.correctedPseudorangeMeters to row
    }

    private fun dopplerFactor(
        state: DoubleArray,
        observation: GnssObservation,
    ): Pair<Double, DoubleArray> {
        val transitSeconds = (observation.correctedPseudorangeMeters - state[6]) / SPEED_OF_LIGHT_MPS
        val satellite = rotateSatellite(observation.satellitePositionMeters, transitSeconds)
        val delta = subtract(state.copyOfRange(0, 3), satellite)
        val distance = norm(delta)
        val unit = DoubleArray(3) { delta[it] / distance }
        val relativeVelocity = DoubleArray(3) {
            state[it + 3] - observation.satelliteVelocityMetersPerSecond[it]
        }
        val radialVelocity = dot(unit, relativeVelocity)
        val row = DoubleArray(STATE_SIZE)
        for (axis in 0..2) row[axis] = (relativeVelocity[axis] - unit[axis] * radialVelocity) / distance
        for (axis in 0..2) row[axis + 3] = unit[axis]
        row[7] = 1.0
        val predicted = radialVelocity + state[7]
        return predicted - observation.correctedRangeRateMetersPerSecond to row
    }

    private fun checkJacobians(
        estimate: PositionEstimate,
        observation: GnssObservation,
    ): Pair<Double, Double> {
        val state = DoubleArray(STATE_SIZE).apply {
            estimate.positionEcefMeters.copyInto(this, 0)
            this[6] = estimate.clockBiasMeters
        }
        return numericalJacobianError(state, observation, ::pseudorangeFactor) to
            if (observation.correctedRangeRateMetersPerSecond.isFinite() &&
                observation.satelliteVelocityMetersPerSecond.all(Double::isFinite)
            ) numericalJacobianError(state, observation, ::dopplerFactor) else Double.NaN
    }

    private fun numericalJacobianError(
        state: DoubleArray,
        observation: GnssObservation,
        factor: (DoubleArray, GnssObservation) -> Pair<Double, DoubleArray>,
    ): Double {
        val analytic = factor(state, observation).second
        var maximum = 0.0
        for (index in state.indices) {
            val epsilon = if (index < 7) 1.0e-3 else 1.0e-4
            val plus = state.copyOf().apply { this[index] += epsilon }
            val minus = state.copyOf().apply { this[index] -= epsilon }
            val numerical = (factor(plus, observation).first - factor(minus, observation).first) /
                (2.0 * epsilon)
            maximum = max(maximum, abs(analytic[index] - numerical))
        }
        return maximum
    }

    private fun computeMetrics(
        estimates: List<PositionEstimate>,
        truth: Map<Long, DoubleArray>,
    ): EstimatorMetrics {
        val horizontalErrors = mutableListOf<Double>()
        val matchedPositions = mutableListOf<DoubleArray>()
        estimates.forEach { estimate ->
            val reference = truth[estimate.timestampMillis]
                ?: truth[estimate.timestampMillis - 1]
                ?: truth[estimate.timestampMillis + 1]
                ?: return@forEach
            val enu = Coordinates.ecefDeltaToEnu(subtract(estimate.positionEcefMeters, reference), reference)
            horizontalErrors += kotlin.math.hypot(enu[0], enu[1])
            matchedPositions += estimate.positionEcefMeters
        }
        val jumps = matchedPositions.zipWithNext { left, right -> norm(subtract(right, left)) }
        val secondDifferences = (2 until matchedPositions.size).map { index ->
            norm(DoubleArray(3) { axis ->
                matchedPositions[index][axis] - 2.0 * matchedPositions[index - 1][axis] +
                    matchedPositions[index - 2][axis]
            })
        }
        return EstimatorMetrics(
            horizontalP50Meters = percentile(horizontalErrors, 50.0),
            horizontalP95Meters = percentile(horizontalErrors, 95.0),
            horizontalMeanMeters = horizontalErrors.average(),
            maxPositionJumpMeters = jumps.maxOrNull() ?: 0.0,
            smoothnessRmsMeters = if (secondDifferences.isEmpty()) 0.0 else
                sqrt(secondDifferences.sumOf { it * it } / secondDifferences.size),
            meanLatencyMillis = estimates.map { it.latencyMillis }.average(),
            solverFailures = estimates.count { !it.converged },
            rejectedMeasurements = estimates.sumOf { it.rejectedCount },
        )
    }

    private fun percentile(values: List<Double>, percentile: Double): Double {
        if (values.isEmpty()) return Double.NaN
        val sorted = values.sorted()
        val position = percentile / 100.0 * (sorted.size - 1)
        val lower = floor(position).toInt()
        val upper = min(lower + 1, sorted.lastIndex)
        val fraction = position - lower
        return sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction
    }

    private fun DoubleArray.toHorizontal(reference: DoubleArray): HorizontalPoint {
        val enu = Coordinates.ecefDeltaToEnu(subtract(this, reference), reference)
        return HorizontalPoint(enu[0], enu[1])
    }

    private fun rotateSatellite(position: DoubleArray, transitSeconds: Double): DoubleArray {
        val angle = EARTH_ROTATION_RADIANS_PER_SECOND * transitSeconds
        val cosine = cos(angle)
        val sine = sin(angle)
        return doubleArrayOf(
            cosine * position[0] + sine * position[1],
            -sine * position[0] + cosine * position[1],
            position[2],
        )
    }

    private fun huberWeight(standardizedResidual: Double, cutoff: Double = 2.5): Double {
        val magnitude = abs(standardizedResidual)
        return if (magnitude <= cutoff) 1.0 else cutoff / magnitude
    }

    private fun dot(left: DoubleArray, right: DoubleArray): Double =
        left.indices.sumOf { left[it] * right[it] }

    private fun Map<String, String>.double(key: String): Double? =
        this[key]?.trim()?.takeIf(String::isNotEmpty)?.toDoubleOrNull()?.takeIf(Double::isFinite)

    private fun Map<String, String>.long(key: String): Long? = double(key)?.toLong()
    private fun Map<String, String>.int(key: String): Int? = double(key)?.toInt()

    companion object {
        private const val SPEED_OF_LIGHT_MPS = 299_792_458.0
        private const val EARTH_ROTATION_RADIANS_PER_SECOND = 7.2921151467e-5
        private const val STATE_SIZE = 8
        private val SUPPORTED_CONSTELLATIONS = setOf(1, 3, 4, 5, 6)

        fun fromStreams(
            deviceGnss: InputStream,
            groundTruth: InputStream,
            lagSeconds: Double = 2.0,
            datasetName: String = "GSDC replay",
            datasetScope: String = "Public derived GNSS replay",
        ): GsdcReplayEstimator = GsdcReplayEstimator(
            CsvReader.read(deviceGnss),
            CsvReader.read(groundTruth),
            lagSeconds,
            datasetName,
            datasetScope,
        )
    }
}
