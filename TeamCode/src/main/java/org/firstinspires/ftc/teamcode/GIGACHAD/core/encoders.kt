package org.firstinspires.ftc.teamcode.GIGACHAD.core


// TODO: Create an suite of testing opmodes that determine all these values empirically
// TODO: Might have to add a separate lateral ticks to handle misalignment
val ENCODER_TICKS_PER_INCH: Double = 0.1;

// Units are in inches, angles in radians

// Expected Encoder setup:
//  ^       ^
//  L   P>  R
//   \  |H /    Distance = D, angle between vertical and perpendicular to this line is phi
//      O

val H: Double = 1.0
val K: Double = 1.0 // k = 1/(2*D*cos(phi))

/**
 *
 *
 * @property height The vertical distance to the line of encoders
 * @property width  The width to the parallel encoders
 * @property inches_per_tick The number of ticks per inch
 * */

data class EncoderValues(
    @JvmField val height: Double,
    @JvmField val width: Double, // k = 1/(2*D*cos(phi))
    @JvmField val inches_per_tick: Double,
)

// TODO: Consider reworking the math here to account for
// slight errors in the actual encoder alignments
data class EncoderReadings(
    @JvmField val left: Double,
    @JvmField val right: Double,
    @JvmField val perp: Double
) {
    fun toPoseVel(values: EncoderValues) = PoseVel(Vector2(
        (perp * values.inches_per_tick) + values.height * (right - left) * values.width,
        (right + left) * 0.5
    ), (right - left) * values.width)
}