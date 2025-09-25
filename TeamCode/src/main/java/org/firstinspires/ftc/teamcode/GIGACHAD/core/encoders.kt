package org.firstinspires.ftc.teamcode.GIGACHAD.core


// TODO: Create an suite of testing opmodes that determine all these values empirically
// TODO: Might have to add a separate lateral ticks to handle misalignment
val ENCODER_TICKS_PER_INCH: Double = 0.1;

// Units are in inches, angles in radians

// Expected Encoder setup:
//  ^       ^
//  L       R
//        /    Distance = D, angle between vertical and perpendicular to this line is phi
//      O
//      | Distance = H
//      |
//      P >

val H: Double = 1.0
val K: Double = 1.0 // k = 1/(2*D*cos(phi))

// TODO: Consider reworking the math here to account for
// slight errors in the actual encoder alignments
data class EncoderReadings(
    @JvmField val left: Double,
    @JvmField val right: Double,
    @JvmField val perp: Double
) {
    fun toPoseVel() = PoseVel(Vector2(
        perp + H * (left - right) * K,
        (right + left) * 0.5
    ), (right - left) * K)
}