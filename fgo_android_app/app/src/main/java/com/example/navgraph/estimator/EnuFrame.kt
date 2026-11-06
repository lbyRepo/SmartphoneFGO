package com.example.navgraph.estimator

import kotlin.math.cos
import kotlin.math.sin

class EnuFrame(referenceEcefMeters: DoubleArray) {
    val referenceEcefMeters = referenceEcefMeters.copyOf()
    private val ecefToEnu: Array<DoubleArray>
    private val enuToEcef: Array<DoubleArray>

    init {
        require(referenceEcefMeters.size == 3)
        val lla = Coordinates.ecefToLla(referenceEcefMeters)
        val latitude = Math.toRadians(lla[0])
        val longitude = Math.toRadians(lla[1])
        ecefToEnu = arrayOf(
            doubleArrayOf(-sin(longitude), cos(longitude), 0.0),
            doubleArrayOf(
                -sin(latitude) * cos(longitude),
                -sin(latitude) * sin(longitude),
                cos(latitude),
            ),
            doubleArrayOf(
                cos(latitude) * cos(longitude),
                cos(latitude) * sin(longitude),
                sin(latitude),
            ),
        )
        enuToEcef = Array(3) { row -> DoubleArray(3) { column -> ecefToEnu[column][row] } }
    }

    fun ecefPositionToEnu(positionEcefMeters: DoubleArray): DoubleArray =
        ecefVectorToEnu(subtract(positionEcefMeters, referenceEcefMeters))

    fun enuPositionToEcef(positionEnuMeters: DoubleArray): DoubleArray {
        val delta = enuVectorToEcef(positionEnuMeters)
        return DoubleArray(3) { referenceEcefMeters[it] + delta[it] }
    }

    fun ecefVectorToEnu(vectorEcef: DoubleArray): DoubleArray = multiply(ecefToEnu, vectorEcef)

    fun enuVectorToEcef(vectorEnu: DoubleArray): DoubleArray = multiply(enuToEcef, vectorEnu)

    fun rangeDerivativeEcefToEnu(derivativeEcef: DoubleArray): DoubleArray =
        DoubleArray(3) { enuAxis ->
            derivativeEcef.indices.sumOf { ecefAxis ->
                derivativeEcef[ecefAxis] * enuToEcef[ecefAxis][enuAxis]
            }
        }

    internal fun ecefToEnuRowMajor(): DoubleArray = DoubleArray(9) { index ->
        ecefToEnu[index / 3][index % 3]
    }

    private fun multiply(matrix: Array<DoubleArray>, vector: DoubleArray): DoubleArray {
        require(vector.size == 3)
        return DoubleArray(3) { row ->
            matrix[row].indices.sumOf { column -> matrix[row][column] * vector[column] }
        }
    }
}
