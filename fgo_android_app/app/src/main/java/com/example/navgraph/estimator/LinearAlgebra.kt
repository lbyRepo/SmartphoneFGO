package com.example.navgraph.estimator

import kotlin.math.abs
import kotlin.math.sqrt

internal fun norm(values: DoubleArray): Double = sqrt(values.sumOf { it * it })

internal fun subtract(left: DoubleArray, right: DoubleArray): DoubleArray =
    DoubleArray(left.size) { left[it] - right[it] }

internal class NormalEquations(private val dimension: Int) {
    private val hessian = Array(dimension) { DoubleArray(dimension) }
    private val gradient = DoubleArray(dimension)

    fun add(row: DoubleArray, residual: Double, scale: Double) {
        for (i in 0 until dimension) {
            val weightedI = row[i] * scale
            if (weightedI == 0.0) continue
            gradient[i] += weightedI * residual * scale
            for (j in i until dimension) {
                val value = weightedI * row[j] * scale
                if (value == 0.0) continue
                hessian[i][j] += value
                if (i != j) hessian[j][i] += value
            }
        }
    }

    fun solve(damping: Double): DoubleArray? {
        val matrix = Array(dimension) { row -> hessian[row].copyOf() }
        val right = DoubleArray(dimension) { -gradient[it] }
        for (index in 0 until dimension) matrix[index][index] += damping
        return solveLinearSystem(matrix, right)
    }

    fun inverseDiagonal(indices: IntArray, damping: Double): DoubleArray? {
        require(indices.all { it in 0 until dimension })
        return DoubleArray(indices.size) { outputIndex ->
            val requestedIndex = indices[outputIndex]
            val matrix = Array(dimension) { row -> hessian[row].copyOf() }
            for (index in 0 until dimension) matrix[index][index] += damping
            val right = DoubleArray(dimension).apply { this[requestedIndex] = 1.0 }
            val column = solveLinearSystem(matrix, right) ?: return null
            column[requestedIndex].takeIf { it.isFinite() && it > 0.0 } ?: return null
        }
    }
}

internal fun solveLinearSystem(matrix: Array<DoubleArray>, right: DoubleArray): DoubleArray? {
    val size = right.size
    for (column in 0 until size) {
        var pivot = column
        for (row in column + 1 until size) {
            if (abs(matrix[row][column]) > abs(matrix[pivot][column])) pivot = row
        }
        if (!matrix[pivot][column].isFinite() || abs(matrix[pivot][column]) < 1.0e-14) return null
        if (pivot != column) {
            val temporaryRow = matrix[column]
            matrix[column] = matrix[pivot]
            matrix[pivot] = temporaryRow
            val temporaryValue = right[column]
            right[column] = right[pivot]
            right[pivot] = temporaryValue
        }
        val divisor = matrix[column][column]
        for (row in column + 1 until size) {
            val factor = matrix[row][column] / divisor
            if (factor == 0.0) continue
            matrix[row][column] = 0.0
            for (entry in column + 1 until size) matrix[row][entry] -= factor * matrix[column][entry]
            right[row] -= factor * right[column]
        }
    }
    val solution = DoubleArray(size)
    for (row in size - 1 downTo 0) {
        var value = right[row]
        for (column in row + 1 until size) value -= matrix[row][column] * solution[column]
        solution[row] = value / matrix[row][row]
        if (!solution[row].isFinite()) return null
    }
    return solution
}
