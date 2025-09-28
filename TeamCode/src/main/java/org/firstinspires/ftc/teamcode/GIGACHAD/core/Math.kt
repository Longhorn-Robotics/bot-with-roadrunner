package org.firstinspires.ftc.teamcode.GIGACHAD.core

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan


fun cot(angleRadians: Double): Double {
    return 1.0 / tan(angleRadians)
}

// ~10 * machine epsilon
private const val EPS = 2.2e-15

/**
 * @usesMathJax
 *
 * Function snz(x) from section VI.A of the [SymForce paper](https://arxiv.org/abs/2204.07889) for use in
 * singularity handling.
 * Nudges values slightly away from zero to avoid singularities
 */
fun snz(x: Double) =
    if (x >= 0.0) {
        EPS
    } else {
        -EPS
    }

fun clamp(x: Double, lo: Double, hi: Double): Double {
    if (x < lo) {
        return lo
    }
    if (x > hi) {
        return hi
    }
    return x
}

/** Rotations represented as complex numbers*/
data class Rotation2d(@JvmField val real: Double, @JvmField val imag: Double) {
    companion object {
        /**
         * Get a `Rotation2d` from an unnormalized angle.
         */
        @JvmStatic
        fun exp(theta: Double) = Rotation2d(cos(theta), sin(theta))

        /**
         * Alias for [exp].
         */
        @JvmStatic
        fun fromDouble(theta: Double) = exp(theta)
    }

    /** Adds an angle to this rotation */
    operator fun plus(x: Double) = this * exp(x)

    /** Rotates a vector*/
    operator fun times(v: Vector2) = Vector2(real * v.x - imag * v.y, imag * v.x + real * v.y)

    /** Composes two rotations */
    operator fun times(r: Rotation2d) =
        Rotation2d(real * r.real - imag * r.imag, real * r.imag + imag * r.real)

    fun vec() = Vector2(real, imag)
    fun inverse() = Rotation2d(real, -imag)

    fun log() = atan2(imag, real)

    fun toDouble() = log()
}

data class Vector2(
    @JvmField val x: Double,
    @JvmField val y: Double,
) {
    operator fun plus(v: Vector2) = Vector2(x + v.x, y + v.y)
    operator fun minus(v: Vector2) = Vector2(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2(-x, -y)

    operator fun times(z: Double) = Vector2(x * z, y * z)
    operator fun div(z: Double) = Vector2(x / z, y / z)

    infix fun dot(v: Vector2) = x * v.x + y * v.y
    fun sqrMag() = this dot this
    fun mag() = sqrt(sqrMag())
    fun norm() = this / mag()

    // Warning: not normalized
    fun angleCast() = Rotation2d(x, y)
}

open class Matrix(@JvmField val rows: Int, @JvmField val cols: Int) {
    protected open val data: DoubleArray = DoubleArray(rows * cols)

    operator fun plus(other: Matrix): Matrix {
        if (rows != other.rows) throw AssertionError()
        if (cols != other.cols) throw AssertionError()

        val ret = Matrix(rows, cols)
        ret.data = data.zip(other.data) {a,b -> a + b} .toDoubleArray()
    }
}
