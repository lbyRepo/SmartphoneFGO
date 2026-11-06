package com.example.navgraph.estimator

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull

class LinearAlgebraTest {
    @Test
    fun inverseDiagonalMatchesIndependentWeightedFactors() {
        val equations = NormalEquations(3)
        equations.add(doubleArrayOf(1.0, 0.0, 0.0), residual = 0.0, scale = 0.5)
        equations.add(doubleArrayOf(0.0, 1.0, 0.0), residual = 0.0, scale = 2.0)
        equations.add(doubleArrayOf(0.0, 0.0, 1.0), residual = 0.0, scale = 4.0)

        val diagonal = requireNotNull(equations.inverseDiagonal(intArrayOf(0, 1, 2), 0.0))
        assertEquals(4.0, diagonal[0], 1.0e-12)
        assertEquals(0.25, diagonal[1], 1.0e-12)
        assertEquals(0.0625, diagonal[2], 1.0e-12)
    }

    @Test
    fun inverseDiagonalRejectsSingularSystem() {
        val equations = NormalEquations(2)
        equations.add(doubleArrayOf(1.0, 0.0), residual = 0.0, scale = 1.0)
        assertNull(equations.inverseDiagonal(intArrayOf(0, 1), 0.0))
    }
}
