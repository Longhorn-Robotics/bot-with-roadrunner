package org.firstinspires.ftc.teamcode.GIGACHAD.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GIGACHAD.hardware.EncoderTuningHardware;

@TeleOp(name = "TeleopSIGMA", group = "Pushbot")
public class LinearEncoderTuning extends OpMode {

    EncoderTuningHardware robot;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("Left Encoder: ", robot.encoderLeft.getCurrentPosition());
        telemetry.addData("Right Encoder: ", robot.encoderRight.getCurrentPosition());
        telemetry.addData("Perp Encoder: ", robot.encoderPerp.getCurrentPosition());
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
