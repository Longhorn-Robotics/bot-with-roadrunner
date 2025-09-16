package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SIGMA.hardware.RobotHardwareSIGMA;
import org.firstinspires.ftc.teamcode.SIGMA.hardware.RobotHardwareTest;
import org.firstinspires.ftc.teamcode.SIGMA.utils.ButtonAction;
import org.firstinspires.ftc.teamcode.SIGMA.utils.PIDController;
import org.firstinspires.ftc.teamcode.SIGMA.utils.TargetedMotor;
@TeleOp(name = "TeleopTest", group = "Pushbot")
public class TeleopTest extends OpMode {

    RobotHardwareTest robot = new RobotHardwareTest();
    Boolean aPressed = false;
    int lastPosition = 0;
    long lastTime = 0;
    double speed;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Hello thomas");
        lastTime = System.nanoTime();
    }
    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if (gamepad1.a) {
            aPressed = !aPressed;
        }
        if (aPressed) {
            robot.motor.setPower(1.0);
        } else {
            robot.motor.setPower(0.0);
        }
        int ticksPerRev = 28;
        double gearRatio = 13.7;
        double ticksPerOutputRev = ticksPerRev * gearRatio;

        int currentPosition = robot.motor.getCurrentPosition();
        int deltaTicks = currentPosition - lastPosition;
        lastPosition = currentPosition;

        double deltaTime = (System.nanoTime() - lastTime) / 1e9; // seconds
        lastTime = System.nanoTime();

        double ticksPerSecond = deltaTicks / deltaTime;
        double rps = ticksPerSecond / ticksPerOutputRev;
        double rpm = rps * 60.0;

        telemetry.addData("Ticks/sec", ticksPerSecond);
        telemetry.addData("RPM", rpm);
        telemetry.update();

    }

}
