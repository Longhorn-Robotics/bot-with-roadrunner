package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SIGMA.hardware.RobotHardwareLite;

@TeleOp(name = "TeleopServoTuner", group = "Pushbot")
public class TeleopServoTuner  extends OpMode {
    static double RAIL_MIN = 0.0f;
    static double RAIL_MAX = 3000.0f;

    final double baseSpeed = 0.001f;
    RobotHardwareLite robot = new RobotHardwareLite();

    //YOUSEF TEST
    double servoPos = 0.0;
    // Code to run once when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting
        //telemetry.addData("Say", "Hello thomas");
        //telemetry.addLine(String.format("Zero Position: %d", robot.rail.getCurrentPosition()));
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
        servoPos += gamepad1.left_stick_y * baseSpeed;
        robot.kicker.setPosition(servoPos);
        telemetry.addData("Servo Position:", servoPos);
        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
