package org.firstinspires.ftc.teamcode.GIGACHAD.opmodes.tuning

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.GIGACHAD.core.EncoderReadings
import org.firstinspires.ftc.teamcode.GIGACHAD.core.Pose
import org.firstinspires.ftc.teamcode.GIGACHAD.core.Pose.Companion.id
import org.firstinspires.ftc.teamcode.GIGACHAD.hardware.EncoderTuningHardware
import kotlin.math.exp


@TeleOp(name = "TeleopSIGMA", group = "Pushbot")
class LinearEncoderTuning : OpMode() {
    var robot: EncoderTuningHardware? = null

    var expectedPose: Pose = id()
    val inchPerTick: Double = 1.0
    var timer: ElapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    var lastTime: Double = timer.time()

    val allHubs = hardwareMap.getAll<LynxModule>(LynxModule::class.java)

    override fun init() {
        robot!!.init(hardwareMap)


        for (hub in allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        }
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    override fun init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY
    override fun start() {
        lastTime = timer.time()
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    override fun loop() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }

        val deltatime = (timer.time() - lastTime) / 1000.0
        lastTime = timer.time()

        telemetry.addData("Left Encoder: ", robot!!.encoderLeft.getCurrentPosition())
        telemetry.addData("Right Encoder: ", robot!!.encoderRight.getCurrentPosition())
        telemetry.addData("Perp Encoder: ", robot!!.encoderPerp.getCurrentPosition())

        val encoderReadings = EncoderReadings(
            inchPerTick * robot!!.encoderLeft.getCurrentPosition().toDouble(),
            inchPerTick * robot!!.encoderRight.getCurrentPosition().toDouble(),
            inchPerTick * robot!!.encoderPerp.getCurrentPosition().toDouble()
        )

        expectedPose += encoderReadings.toPoseVel() * deltatime

        telemetry.addData("Tracked Position: ", expectedPose)
    }

    // Code to run ONCE after the driver hits STOP
    override fun stop() {
    }
}
