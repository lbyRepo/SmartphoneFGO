package com.example.navgraph.estimator

class SyntheticGnssInsBenchmark {
    fun run(): GnssInsValidation {
        val dataset = SyntheticGnssInsDatasetFactory.create()
        val wls = GnssWlsSolver(applyEarthRotation = false).solve(dataset.gnssEpochs)
        require(wls.all { it.converged }) { "Synthetic WLS did not converge" }
        val (ekfEstimates, ekf) = runEkf(dataset, wls)
        val smoother = InertialFixedLagSmoother(
            dataset.frame,
            lagSeconds = 2.0,
            applyEarthRotation = false,
        ).run(dataset.gnssEpochs, dataset.imuSamples, wls)
        val jacobians = GnssInsJacobianChecker.check(
            dataset.frame,
            ekf.state,
            dataset.gnssEpochs.last().observations.first(),
            applyEarthRotation = false,
        )
        require(jacobians.first < 2.0e-5 && jacobians.second < 2.0e-5) {
            "Synthetic GNSS/INS Jacobian check failed: $jacobians"
        }
        val reference = dataset.frame.referenceEcefMeters
        return GnssInsValidation(
            durationSeconds = (dataset.imuSamples.last().timestampNanos -
                dataset.imuSamples.first().timestampNanos) * 1.0e-9,
            imuSampleCount = dataset.imuSamples.size,
            gnssEpochCount = dataset.gnssEpochs.size,
            inertialFactorCount = smoother.inertialFactorCount,
            wlsMetrics = EstimatorMetricsCalculator.compute(wls, dataset.truthByTimestampMillis),
            ekfMetrics = EstimatorMetricsCalculator.compute(ekfEstimates, dataset.truthByTimestampMillis),
            fgoMetrics = EstimatorMetricsCalculator.compute(smoother.estimates, dataset.truthByTimestampMillis),
            truthTrajectory = dataset.truthByTimestampMillis.values.map { it.toPoint(reference) },
            wlsTrajectory = wls.map { it.positionEcefMeters.toPoint(reference) },
            ekfTrajectory = ekfEstimates.map { it.positionEcefMeters.toPoint(reference) },
            fgoTrajectory = smoother.estimates.map { it.positionEcefMeters.toPoint(reference) },
            ekfAcceptedPseudorange = ekf.health.acceptedPseudorange,
            ekfRejectedPseudorange = ekf.health.rejectedPseudorange,
            ekfAcceptedDoppler = ekf.health.acceptedDoppler,
            ekfRejectedDoppler = ekf.health.rejectedDoppler,
            finalPositionSigmaMeters = ekf.positionStandardDeviationMeters(),
            finalFgoPositionSigmaMeters = smoother.stateUncertainties.lastOrNull()
                ?.positionSigmaEnuMeters?.copyOf() ?: DoubleArray(3) { Double.NaN },
            pseudorangeJacobianError = jacobians.first,
            dopplerJacobianError = jacobians.second,
        )
    }

    private fun runEkf(
        dataset: SyntheticGnssInsDataset,
        wls: List<PositionEstimate>,
    ): Pair<List<PositionEstimate>, GnssInsEkf> {
        val firstWls = wls.first()
        val initialState = InsState(
            timestampNanos = dataset.imuSamples.first().timestampNanos,
            positionEnuMeters = dataset.frame.ecefPositionToEnu(firstWls.positionEcefMeters),
            velocityEnuMetersPerSecond = dataset.frame.ecefVectorToEnu(
                firstWls.velocityEcefMetersPerSecond,
            ),
            attitudeBodyToEnu = UnitQuaternion.IDENTITY,
            accelerometerBiasBodyMetersPerSecondSquared = DoubleArray(3),
            gyroBiasBodyRadiansPerSecond = DoubleArray(3),
            clockBiasMeters = firstWls.clockBiasMeters,
            clockDriftMetersPerSecond = firstWls.clockDriftMetersPerSecond,
        )
        val ekf = GnssInsEkf(
            dataset.frame,
            initialState,
            applyEarthRotation = false,
        )
        val estimates = mutableListOf<PositionEstimate>()
        var previousRejected = 0

        fun updateAndRecord(epoch: GnssEpoch, latencyStartedNanos: Long) {
            ekf.update(epoch)
            val rejected = ekf.health.rejectedMeasurements - previousRejected
            previousRejected = ekf.health.rejectedMeasurements
            estimates += PositionEstimate(
                timestampMillis = epoch.timestampMillis,
                positionEcefMeters = dataset.frame.enuPositionToEcef(ekf.state.positionEnuMeters),
                velocityEcefMetersPerSecond = dataset.frame.enuVectorToEcef(
                    ekf.state.velocityEnuMetersPerSecond,
                ),
                clockBiasMeters = ekf.state.clockBiasMeters,
                clockDriftMetersPerSecond = ekf.state.clockDriftMetersPerSecond,
                iterations = 1,
                observationCount = epoch.observations.size,
                rejectedCount = rejected,
                latencyMillis = (System.nanoTime() - latencyStartedNanos) / 1.0e6,
                converged = ekf.positionStandardDeviationMeters().all(Double::isFinite),
            )
        }

        updateAndRecord(dataset.gnssEpochs.first(), System.nanoTime())
        val epochsByTimestampNanos = dataset.gnssEpochs.drop(1).associateBy {
            it.integrationTimestampNanos
        }
        var previous = dataset.imuSamples.first()
        for (index in 1 until dataset.imuSamples.size) {
            val current = dataset.imuSamples[index]
            ekf.propagate(previous, current)
            epochsByTimestampNanos[current.timestampNanos]?.let { epoch ->
                updateAndRecord(epoch, System.nanoTime())
            }
            previous = current
        }
        require(estimates.size == dataset.gnssEpochs.size) { "Not every synthetic GNSS epoch was processed" }
        return estimates to ekf
    }

    private fun DoubleArray.toPoint(referenceEcef: DoubleArray): HorizontalPoint {
        val enu = Coordinates.ecefDeltaToEnu(subtract(this, referenceEcef), referenceEcef)
        return HorizontalPoint(enu[0], enu[1])
    }
}
