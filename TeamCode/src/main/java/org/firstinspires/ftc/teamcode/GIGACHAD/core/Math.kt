package org.firstinspires.ftc.teamcode.GIGACHAD.core

import android.annotation.SuppressLint
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

/** Rotations represented as complex numbers
 * Advanced: The names of Log and Exp aren't the typical math functions, they're from Lie theory of the SO(2) group*/
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

data class Vector2(@JvmField val x: Double, @JvmField val y: Double) {

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


data class Vector3(@JvmField val x: Double, @JvmField val y: Double, @JvmField val z: Double) {
    operator fun plus(v: Vector3) = Vector3(x + v.x, y + v.y, z + v.z)
    operator fun minus(v: Vector3) = Vector3(x - v.x, y - v.y, z - v.z)
    operator fun unaryMinus() = Vector3(-x, -y, -z)

    operator fun times(s: Double) = Vector3(x * s, y * s, z * s)
    operator fun div(s: Double) = Vector3(x / s, y / s, z / s)

    infix fun dot(v: Vector3) = x * v.x + y * v.y + z * v.z
    infix fun cross(v: Vector3) = Vector3(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    )
    fun sqrMag() = this dot this
    fun mag() = sqrt(sqrMag())
    fun norm() = this / mag()

    // Warning: not normalized
    fun angleCast() = Rotation2d(x, y)
}


/**
 * Represents a mathematical matrix with double-precision floating-point elements.
 * The elements are stored in a single DoubleArray in row-major order.
 *
 * @property rows The number of rows in the matrix.
 * @property cols The number of columns in the matrix.
 * @property data The internal DoubleArray storing the matrix elements in row-major order.
 */
class Matrix(val rows: Int, val cols: Int, val data: DoubleArray) {

    // Primary constructor validation
    init {
        require(rows > 0 && cols > 0) { "Matrix dimensions must be positive." }
        require(data.size == rows * cols) { "Data array size must match rows * cols." }
    }

    // Secondary constructor to create a matrix from a 2D array (for convenience)
    constructor(array2D: Array<DoubleArray>) :
            this(
                rows = array2D.size,
                cols = if (array2D.isNotEmpty()) array2D[0].size else 0,
                data = array2D.flatMap { it.asIterable() }.toDoubleArray()
            ) {
        // Validate that all rows have the same number of columns
        val expectedCols = if (array2D.isNotEmpty()) array2D[0].size else 0
        require(array2D.all { it.size == expectedCols }) { "All rows in the 2D array must have the same length." }
    }

    companion object {
        @JvmStatic
        fun Identity(size: Int): Matrix {
            require(size > 0) { "Matrices must have positive dimensions" }
            val arr = DoubleArray(size * size);
            for (i in 0 until size - 1) {
                arr[i * (size + 1)] = 1.0
            }
            return Matrix(size, size, arr)
        }

        @JvmStatic
        fun SkewSymetric(size: Int) {}
    }

    /**
     * Calculates the 1D index in the 'data' array for a given (row, col) pair.
     * Uses row-major order: index = row * cols + col.
     */
    private fun getIndex(row: Int, col: Int): Int {
        return row * cols + col
    }

    // --- Element Access (Operator Overloading) ---

    /**
     * Gets the element at the specified (row, col).
     * Usage: val element = matrix[i, j]
     */
    operator fun get(row: Int, col: Int): Double = data[getIndex(row, col)]

    /**
     * Sets the element at the specified (row, col).
     * Usage: matrix[i, j] = value
     */
    operator fun set(row: Int, col: Int, value: Double) { data[getIndex(row, col)] = value }

    // --- Basic Matrix Operations ---

    /**
     * Performs matrix addition: A + B.
     */
    operator fun plus(other: Matrix): Matrix {
        require(rows == other.rows && cols == other.cols) { "Matrices must have the same dimensions for addition." }

        val newData = DoubleArray(data.size) { i -> this.data[i] + other.data[i] }
        return Matrix(rows, cols, newData)
    }

    /**
     * Performs matrix subtraction: A - B.
     */
    operator fun minus(other: Matrix): Matrix {
        require(rows == other.rows && cols == other.cols) { "Matrices must have the same dimensions for subtraction." }

        val newData = DoubleArray(data.size) { i -> this.data[i] - other.data[i] }
        return Matrix(rows, cols, newData)
    }

    /**
     * Performs matrix multiplication: A * B.
     * The number of columns in this matrix (A) must equal the number of rows in 'other' (B).
     */
    operator fun times(other: Matrix): Matrix {
        require(cols == other.rows) { "Number of columns in first matrix must equal number of rows in second matrix for multiplication." }

        val resultRows = this.rows
        val resultCols = other.cols
        val resultData = DoubleArray(resultRows * resultCols)

        // Oooh, O(n^3), So Scary
        for (i in 0 until resultRows) { // A row index
            for (j in 0 until resultCols) { // B column index
                var sum = 0.0
                for (k in 0 until this.cols) { // Inner dimension
                    // result[i, j] += A[i, k] * B[k, j]
                    sum += this[i, k] * other[k, j]
                }
                resultData[i * resultCols + j] = sum
            }
        }
        return Matrix(resultRows, resultCols, resultData)
    }

    /**
     * Multiplies the matrix by a scalar.
     */
    operator fun times(scalar: Double): Matrix {
        val newData = DoubleArray(data.size) { i -> this.data[i] * scalar }
        return Matrix(rows, cols, newData)
    }

    /**
     * Multiplies the matrix by a column Vector2. Must be a 2x2 matrix
     */
    operator fun times(v: Vector2): Vector2 {
        require(rows == 2 && cols == 2) { "Matrix must be 2x2" }
        return Vector2(
            v.x * data[0] + v.y * data[1],
            v.x * data[2] + v.y * data[3])
    }

    /**
     * Multiplies the matrix by a column Vector3. Must be a 2x2 matrix
     */
    operator fun times(v: Vector3): Vector3 {
        require(rows == 3 && cols == 3) { "Matrix must be 2x2" }
        return Vector3(
            v.x * data[0] + v.y * data[1] + v.z * data[2],
            v.x * data[3] + v.y * data[4] + v.z * data[5],
            v.x * data[7] + v.y * data[5] + v.z * data[5],)

    }

    // --- Other Essential Linear Algebra Operations ---

    /**
     * Calculates the transpose of the matrix: A^T.
     */
    fun transpose(): Matrix {
        val resultRows = this.cols
        val resultCols = this.rows
        val resultData = DoubleArray(data.size)

        for (i in 0 until rows) {
            for (j in 0 until cols) {
                // A[i, j] becomes A_T[j, i]
                resultData[j * resultCols + i] = this[i, j]
            }
        }
        return Matrix(resultRows, resultCols, resultData)
    }

    // --- Utility Function for Debugging/Printing ---

    @SuppressLint("DefaultLocale")
    override fun toString(): String {
        val sb = StringBuilder()
        sb.append("Matrix ($rows x $cols):\n")
        for (i in 0 until rows) {
            sb.append("[ ")
            for (j in 0 until cols) {
                sb.append(String.format("%.4f", this[i, j])).append(if (j < cols - 1) ", " else "")
            }
            sb.append(" ]\n")
        }
        return sb.toString()
    }
}
