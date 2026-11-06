package com.example.navgraph.estimator

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class UnitQuaternion(
    val w: Double,
    val x: Double,
    val y: Double,
    val z: Double,
) {
    fun normalized(): UnitQuaternion {
        val magnitude = sqrt(w * w + x * x + y * y + z * z)
        require(magnitude > 1.0e-15) { "Cannot normalize a zero quaternion" }
        return UnitQuaternion(w / magnitude, x / magnitude, y / magnitude, z / magnitude)
    }

    operator fun times(other: UnitQuaternion): UnitQuaternion = UnitQuaternion(
        w * other.w - x * other.x - y * other.y - z * other.z,
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w,
    ).normalized()

    fun conjugate(): UnitQuaternion = UnitQuaternion(w, -x, -y, -z)

    fun rotate(vector: DoubleArray): DoubleArray {
        require(vector.size == 3)
        val matrix = rotationMatrix()
        return DoubleArray(3) { row ->
            matrix[row].indices.sumOf { column -> matrix[row][column] * vector[column] }
        }
    }

    fun rotationMatrix(): Array<DoubleArray> {
        val q = normalized()
        val xx = q.x * q.x
        val yy = q.y * q.y
        val zz = q.z * q.z
        val xy = q.x * q.y
        val xz = q.x * q.z
        val yz = q.y * q.z
        val wx = q.w * q.x
        val wy = q.w * q.y
        val wz = q.w * q.z
        return arrayOf(
            doubleArrayOf(1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            doubleArrayOf(2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            doubleArrayOf(2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )
    }

    companion object {
        val IDENTITY = UnitQuaternion(1.0, 0.0, 0.0, 0.0)

        fun fromRotationVector(rotationRadians: DoubleArray): UnitQuaternion {
            require(rotationRadians.size == 3)
            val angle = norm(rotationRadians)
            if (angle < 1.0e-12) {
                return UnitQuaternion(
                    1.0,
                    0.5 * rotationRadians[0],
                    0.5 * rotationRadians[1],
                    0.5 * rotationRadians[2],
                ).normalized()
            }
            val halfAngle = 0.5 * angle
            val scale = sin(halfAngle) / angle
            return UnitQuaternion(
                cos(halfAngle),
                rotationRadians[0] * scale,
                rotationRadians[1] * scale,
                rotationRadians[2] * scale,
            )
        }
    }
}
