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

// Checklist of operators:
// Exp and Log
// identity
// times for Pose, Vel + Vel Addition/subtraction
// Pose . Vel left/right addition/subtraction

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

    companion object {
        @JvmStatic
        fun id(): Pose = Pose(Vector2(0.0, 0.0), Rotation2d(1.0, 0.0))

        @JvmStatic
        fun Exp(t: PoseVel): Pose {

            // Here we have to deal with a singularity as angularVel -> 0.0
            // In the math, that gives us an indeterminate form (0/0) for
            // which the limit gives us 1. The snz function handles this nicely.

            val w = t.angularVel + snz(t.angularVel)
            val c = 1 - cos(w)
            val s = sin(w)
            val translation = Vector2(
                (s * t.linearVel.x - c * t.linearVel.y) / w,
                (c * t.linearVel.x + s * t.linearVel.y) / w,
            )
            val heading = Rotation2d(1 - c, s)

            return Pose(translation, heading)
        }

    }

    operator fun times(p: Pose) = Pose(heading * p.position + position, heading * p.heading)
    operator fun times(v: Vector2) = heading * v + position

    // Right +
    operator fun plus(t: PoseVel): Pose = this * Pose.Exp(t)

    // Right -
    operator fun minus(p: Pose): PoseVel = (this.inverse() * p).Log()

    fun inverse() = Pose(heading.inverse() * -position, heading.inverse())

    // Returns the *principle* logarithm; That is, the PoseVel
    // with the smallest angular velocity possible
    fun Log(): PoseVel {
        // Here is where the multi-valued comes in, we could make omega any whole number of
        // rotations away
        val omega = heading.log()

        val halfw = 0.5 * omega + snz(omega)
        val ct = cot(halfw)
        val linearVel = Vector2(
            (ct * position.x + position.y) * halfw,
            (-position.x + ct * position.y) * halfw
        )

        return PoseVel(linearVel, omega)
    }
}


data class PoseVel(@JvmField val linearVel: Vector2, @JvmField val angularVel: Double) {
    companion object {
        @JvmStatic
        fun id(): PoseVel = PoseVel(Vector2(0.0, 0.0), 0.0)
    }

    operator fun plus(t: PoseVel) = PoseVel(linearVel + t.linearVel, angularVel + t.angularVel)
    operator fun minus(t: PoseVel) = PoseVel(linearVel - t.linearVel, angularVel - t.angularVel)
    operator fun unaryMinus() = PoseVel(-linearVel, -angularVel)

    operator fun times(t: Double) = PoseVel(linearVel * t, angularVel * t)
}