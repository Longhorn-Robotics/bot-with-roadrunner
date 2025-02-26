package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SIGMA.hardware.RobotHardwareIntakeCalibration;

@TeleOp(name = "TeleopIntakeCalibration", group = "Pushbot")
public class TeleopIntakeCalibration extends OpMode {

    static final double RAIL_MIN = 10.0f;
    static final double RAIL_MAX = 3000.0f;
    static final double CLAW_OPEN = 0.0f;
    static final double CLAW_CLOSED = 0.75f;
    static final double ARM_DOWN = 5.0f;
    static final double ARM_UP = 673.0f;

    /* Declare OpMode members. */
    RobotHardwareIntakeCalibration robot = new RobotHardwareIntakeCalibration();
    double pinchPosition = 0.0;
//    double extendPosition = 1.0;
//    double pivotPosition = 0.0;
//    double bucketLPosition = 1;
//    double bucketRPosition = 1;

//    double aidensDrivingTheBusNow = 0;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Hello thomas");

        pinchPosition = robot.pincher.getPosition();
//        extendPosition = robot.extender.getPosition();
//        pivotPosition = robot.pivot.getPosition();
//        bucketLPosition = robot.bucketL.getPosition();
//        bucketRPosition = robot.bucketR.getPosition();
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
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {


        final double x = 0.002;

        if (gamepad1.dpad_right) pinchPosition += 0.002;
        if (gamepad1.dpad_left) pinchPosition -= 0.002;
//
//        if (gamepad1.dpad_up) extendPosition -= 0.002;
//        if (gamepad1.dpad_down) extendPosition += 0.002;
//
//        if (gamepad1.left_bumper) pivotPosition += 0.002;
//        if (gamepad1.right_bumper) pivotPosition -= 0.002;


//        if (gamepad1.dpad_up) bucketLPosition += x;
//        if (gamepad1.dpad_down) bucketLPosition -= x;

//        if (gamepad1.left_bumper) bucketRPosition += x;
//        if (gamepad1.right_bumper) bucketRPosition -= x;

        telemetry.addLine(String.format("Pinch Position: %6.2f", pinchPosition));
//        telemetry.addLine(String.format("Extend Position: %6.2f", extendPosition));
//        telemetry.addLine(String.format("Pivot Position: %6.2f", pivotPosition));
//        telemetry.addLine(String.format("Left Position: %6.2f", bucketLPosition));
//        telemetry.addLine(String.format("Right Position: %6.2f", bucketRPosition));

        // pivot down: 0.04
        // pivot floating: 0.20
        // pivot back: 0.90

        // pinch open: 0.66
        // pinch closed: 0.89

        // extend out: 0.12
        // extend in: 0.42

        // bucket dump R: 1.0
        // bucket pick R: 0.6

        robot.pincher.setPosition(pinchPosition);
//        robot.bucketL.setPosition(bucketLPosition);
//        robot.extender.setPosition(extendPosition);

//        if (gamepad1.dpad_up) bucketLPosition -= 0.002;
//        if (gamepad1.dpad_down) bucketLPosition += 0.002;

//        if (gamepad1.dpad_right) bucketRPosition -= 0.002;
//        if (gamepad1.dpad_left) bucketRPosition += 0.002;

//        if (gamepad1.dpad_left) aidensDrivingTheBusNow -= 0.002;
//        if (gamepad1.dpad_right) aidensDrivingTheBusNow += 0.002;

        // L: 1;
        // R: 0.956

//        robot.bucketL.setPosition(bucketLPosition);
//        robot.bucketR.setPosition(bucketRPosition);

//        robot.specimenGrabber.setPosition(aidensDrivingTheBusNow);
//        telemetry.addLine(String.format("SpecimenGrabber: %f",robot.specimenGrabber.getPosition()));

//        telemetry.addLine(String.format("BucketL: %f", robot.bucketL.getPosition()));
//        telemetry.addLine(String.format("BucketR: %f", robot.bucketR.getPosition()));

        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}

