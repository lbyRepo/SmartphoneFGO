package com.example.navgraph.estimator

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class ImuMechanizationTest {
    @Test
    fun quaternionRotatesBodyVectorIntoNavigationFrame() {
        val quaternion = UnitQuaternion.fromRotationVector(doubleArrayOf(0.0, 0.0, Math.PI / 2.0))
        val rotated = quaternion.rotate(doubleArrayOf(1.0, 0.0, 0.0))
        assertEquals(0.0, rotated[0], 1.0e-12)
        assertEquals(1.0, rotated[1], 1.0e-12)
        assertEquals(0.0, rotated[2], 1.0e-12)
    }

    @Test
    fun stationarySpecificForceCancelsGravity() {
        val first = ImuSample(
            1_000_000_000L,
            doubleArrayOf(0.0, 0.0, ImuMechanization.STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED),
            DoubleArray(3),
        )
        val second = first.copy(timestampNanos = 2_000_000_000L)
        val initial = InsState(
            timestampNanos = first.timestampNanos,
            positionEnuMeters = DoubleArray(3),
            velocityEnuMetersPerSecond = DoubleArray(3),
            attitudeBodyToEnu = UnitQuaternion.IDENTITY,
            accelerometerBiasBodyMetersPerSecondSquared = DoubleArray(3),
            gyroBiasBodyRadiansPerSecond = DoubleArray(3),
            clockBiasMeters = 10.0,
            clockDriftMetersPerSecond = 0.5,
        )
        val propagated = ImuMechanization.propagate(initial, first, second)
        assertTrue(propagated.positionEnuMeters.all { kotlin.math.abs(it) < 1.0e-12 })
        assertTrue(propagated.velocityEnuMetersPerSecond.all { kotlin.math.abs(it) < 1.0e-12 })
        assertEquals(10.5, propagated.clockBiasMeters, 1.0e-12)
    }

    @Test
    fun gyroPropagationAppliesBodyToNavigationRotation() {
        val first = ImuSample(
            1_000_000_000L,
            doubleArrayOf(0.0, 0.0, ImuMechanization.STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED),
            doubleArrayOf(0.0, 0.0, Math.PI / 2.0),
        )
        val second = first.copy(timestampNanos = 2_000_000_000L)
        val initial = InsState(
            timestampNanos = first.timestampNanos,
            positionEnuMeters = DoubleArray(3),
            velocityEnuMetersPerSecond = DoubleArray(3),
            attitudeBodyToEnu = UnitQuaternion.IDENTITY,
            accelerometerBiasBodyMetersPerSecondSquared = DoubleArray(3),
            gyroBiasBodyRadiansPerSecond = DoubleArray(3),
            clockBiasMeters = 0.0,
            clockDriftMetersPerSecond = 0.0,
        )
        val propagated = ImuMechanization.propagate(initial, first, second)
        val rotated = propagated.attitudeBodyToEnu.rotate(doubleArrayOf(1.0, 0.0, 0.0))
        assertEquals(0.0, rotated[0], 1.0e-12)
        assertEquals(1.0, rotated[1], 1.0e-12)
    }
}
