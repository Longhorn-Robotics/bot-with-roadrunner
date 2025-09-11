package org.firstinspires.ftc.teamcode.GIGACHAD.core

import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt



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

/**
 * @usesMathJax
 *
 * 2D rigid transform comprised of [heading] followed by [position].
 *
 * The pose `destPoseSource` denotes the transform from frame `Source` into frame `Dest`. It can be applied with
 * `times()` to change the coordinates of `xSource` into `xDest` where `x` is a vector, twist, or even another pose:
 * `xDest = destPoseSource * xSource`. The awkward names take some getting used to, but they avoid many routine errors.
 *
 *  That means p1 * p2 is intuitively the same as applying p1, then p2 from p1's state
 *  Say p1 is 10 x, 5 y, angle of 45
 *  p2 is 3 x, 2 y, angle of -45
 *  p1 * p2 ~= 10.7 x, 8.5 y, angle of 0
 *
 * Transforms into the world frame are common enough to warrant a shorthand. The pose `worldPoseSource` can be shortened
 * to `poseSource` for any frame `Source`.
 *
 * Advanced: Transforms in two dimensions comprise a Lie group referred to as SE(2). The terminology [exp] and [log]
 * comes from the Lie theory, and [this paper](https://arxiv.org/abs/1812.01537) gives a targeted exposition of the key
 * fundamentals.
 *
 * Twist is the R^3 representation (vee) of the Lie Algebra of Pose: Pose = SE(2), Twist^ = se(2)
 * Twist is essentially a representation of the (both linear and angular) velocity in terms of Pose
 * Thus the `PoseVelocity2d` that roadrunner uses shouldn't actually be necessary
 */
data class Pose(
    @JvmField val position: Vector2,
    @JvmField val heading: Rotation2d,
) {

    operator fun times(p: Pose) = Pose(heading * p.position + position, heading * p.heading)
    operator fun times(v: Vector2) = heading * v + position

    fun inverse() = Pose(heading.inverse() * -position, heading.inverse())
}


data class Twist(@JvmField val line: Vector2d, @JvmField val angle: Double)