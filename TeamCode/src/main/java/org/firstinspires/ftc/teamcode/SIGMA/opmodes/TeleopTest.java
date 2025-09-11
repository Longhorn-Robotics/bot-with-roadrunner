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
    double speed;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Hello thomas");
    }
    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        robot.motor.setPower(gamepad1.left_stick_y * 30);
        speed = robot.motor.getVelocity() * 60 / (384.5 * 4);
        telemetry.addData("Speed", speed + " RPM");
        telemetry.update();
    }

}
