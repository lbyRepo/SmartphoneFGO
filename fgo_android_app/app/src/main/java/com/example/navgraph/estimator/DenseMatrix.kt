package com.example.navgraph.estimator

internal fun identityMatrix(size: Int): Array<DoubleArray> =
    Array(size) { row -> DoubleArray(size).apply { this[row] = 1.0 } }

internal fun multiplyMatrices(
    left: Array<DoubleArray>,
    right: Array<DoubleArray>,
): Array<DoubleArray> {
    require(left.isNotEmpty() && right.isNotEmpty() && left[0].size == right.size)
    return Array(left.size) { row ->
        DoubleArray(right[0].size) { column ->
            var sum = 0.0
            for (inner in right.indices) sum += left[row][inner] * right[inner][column]
            sum
        }
    }
}

internal fun transposeMatrix(matrix: Array<DoubleArray>): Array<DoubleArray> =
    Array(matrix[0].size) { row -> DoubleArray(matrix.size) { column -> matrix[column][row] } }

internal fun symmetrize(matrix: Array<DoubleArray>) {
    for (row in matrix.indices) {
        for (column in row + 1 until matrix.size) {
            val average = 0.5 * (matrix[row][column] + matrix[column][row])
            matrix[row][column] = average
            matrix[column][row] = average
        }
        if (matrix[row][row] < 1.0e-15) matrix[row][row] = 1.0e-15
    }
}

internal fun skew(vector: DoubleArray): Array<DoubleArray> = arrayOf(
    doubleArrayOf(0.0, -vector[2], vector[1]),
    doubleArrayOf(vector[2], 0.0, -vector[0]),
    doubleArrayOf(-vector[1], vector[0], 0.0),
)
