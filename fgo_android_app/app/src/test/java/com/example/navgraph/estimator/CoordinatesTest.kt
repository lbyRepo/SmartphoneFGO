package com.example.navgraph.estimator

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class CoordinatesTest {
    @Test
    fun llaEcefRoundTripAtSingapore() {
        val expected = doubleArrayOf(1.3521, 103.8198, 42.0)
        val recovered = Coordinates.ecefToLla(
            Coordinates.llaToEcef(expected[0], expected[1], expected[2]),
        )
        expected.indices.forEach { index ->
            assertEquals(expected[index], recovered[index], 1.0e-7)
        }
    }

    @Test
    fun enuAxesAtEquatorPrimeMeridian() {
        val reference = Coordinates.llaToEcef(0.0, 0.0, 0.0)
        val east = Coordinates.ecefDeltaToEnu(doubleArrayOf(0.0, 1.0, 0.0), reference)
        val north = Coordinates.ecefDeltaToEnu(doubleArrayOf(0.0, 0.0, 1.0), reference)
        assertTrue(east.contentEqualsWithin(doubleArrayOf(1.0, 0.0, 0.0), 1.0e-12))
        assertTrue(north.contentEqualsWithin(doubleArrayOf(0.0, 1.0, 0.0), 1.0e-12))
    }

    private fun DoubleArray.contentEqualsWithin(other: DoubleArray, tolerance: Double): Boolean =
        indices.all { kotlin.math.abs(this[it] - other[it]) <= tolerance }
}
