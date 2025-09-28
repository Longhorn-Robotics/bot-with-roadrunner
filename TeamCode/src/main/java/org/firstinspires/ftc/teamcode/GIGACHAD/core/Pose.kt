package org.firstinspires.ftc.teamcode.GIGACHAD.core

import kotlin.math.cos
import kotlin.math.sin

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