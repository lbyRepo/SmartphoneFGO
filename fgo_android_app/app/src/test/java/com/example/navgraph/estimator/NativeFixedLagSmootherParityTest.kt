package com.example.navgraph.estimator

import java.io.File
import org.junit.Assume.assumeTrue
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class NativeFixedLagSmootherParityTest {
    @Test
    fun jniMatchesKotlinWindowStatesIterationsAndMarginals() {
        assertWindowParity(epochCount = 3)
    }

    @Test
    fun jniMatchesKotlinAtTheFiveSecondWindowSize() {
        assertWindowParity(epochCount = 6)
    }

    private fun assertWindowParity(epochCount: Int) {
        val library = File(
            System.getProperty("java.io.tmpdir"),
            "navgraph_native_parity/libnavgraph_estimator.dylib",
        )
        assumeTrue("Native parity library is not built: ${library.absolutePath}", library.isFile)
        NativeFixedLagSmoother.loadAbsoluteForTests(library.absolutePath)
        val dataset = SyntheticGnssInsDatasetFactory.create(seed = 123L)
        val epochs = dataset.gnssEpochs.take(epochCount)
        val solver = GnssWlsSolver(applyEarthRotation = false)
        val wls = mutableListOf<PositionEstimate>()
        epochs.forEach { epoch -> wls += solver.solveEpoch(epoch, wls.lastOrNull()) }
        assertTrue(wls.all { it.converged })
        val preintegrations = epochs.zipWithNext { first, second ->
            ImuMechanization.preintegrate(
                dataset.imuSamples.filter {
                    it.timestampNanos in first.integrationTimestampNanos..second.integrationTimestampNanos
                },
                UnitQuaternion.IDENTITY,
            )
        }
        val kotlin = InertialFixedLagSmoother(
            dataset.frame,
            lagSeconds = FgoConfig.DEFAULT_LAG_SECONDS,
            maxKeyframes = FgoConfig.MAX_KEYFRAMES,
            applyEarthRotation = false,
        ).solveWindow(epochs, wls, preintegrations)
        val native = assertNotNull(
            NativeFixedLagSmoother.solveWindow(
                dataset.frame,
                epochs,
                wls,
                preintegrations,
                applyEarthRotation = false,
            ),
        )
        assertEquals(NativeFgoStatus.OK, native.status)
        assertEquals(kotlin.converged, native.converged)
        assertEquals(kotlin.iterations, native.iterations)
        assertEquals(kotlin.states.size, native.states.size)
        kotlin.states.indices.forEach { stateIndex ->
            kotlin.states[stateIndex].indices.forEach { component ->
                assertEquals(
                    kotlin.states[stateIndex][component],
                    native.states[stateIndex][component],
                    2.0e-7,
                    "state[$stateIndex][$component]",
                )
            }
        }
        val kotlinUncertainty = assertNotNull(kotlin.uncertainty)
        val nativeUncertainty = assertNotNull(native.uncertainty)
        for (axis in 0..2) {
            assertEquals(
                kotlinUncertainty.positionSigmaEnuMeters[axis],
                nativeUncertainty.positionSigmaEnuMeters[axis],
                2.0e-7,
            )
            assertEquals(
                kotlinUncertainty.velocitySigmaEnuMetersPerSecond[axis],
                nativeUncertainty.velocitySigmaEnuMetersPerSecond[axis],
                2.0e-7,
            )
        }
        assertEquals(kotlinUncertainty.clockBiasSigmaMeters,
            nativeUncertainty.clockBiasSigmaMeters, 2.0e-7)
        assertEquals(kotlinUncertainty.clockDriftSigmaMetersPerSecond,
            nativeUncertainty.clockDriftSigmaMetersPerSecond, 2.0e-7)
    }
}
