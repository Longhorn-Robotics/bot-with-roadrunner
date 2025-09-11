package org.firstinspires.ftc.teamcode.GIGACHAD.core

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class Rotation2d(@JvmField val real: Double, @JvmField val imag: Double) {
    companion object {
        @JvmStatic
        fun exp(theta: Double) = Rotation2d(cos(theta), sin(theta))
    }
}

data class Vector2d(
    @JvmField val x: Double,
    @JvmField val y: Double,
    ) {
    operator fun plus(v: Vector2d) = Vector2d(x + v.x, y + v.y)
    operator fun minus(v: Vector2d) = Vector2d(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2d(-x, -y)

    operator fun times(z: Double) = com.acmerobotics.roadrunner.Vector2d(x * z, y * z)
    operator fun div(z: Double) = com.acmerobotics.roadrunner.Vector2d(x / z, y / z)

    infix fun dot(v: Vector2d) = x * v.x + y * v.y
    fun sqrMag() = this dot this
    fun mag() = sqrt(sqrMag())
    fun norm() = this / mag()

    // Warning: not normalized
    fun angleCast() = Rotation2d(x, y)
}


