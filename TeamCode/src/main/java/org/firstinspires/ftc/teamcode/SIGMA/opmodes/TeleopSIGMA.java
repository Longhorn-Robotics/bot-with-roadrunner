package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SIGMA.hardware.RobotHardwareSIGMA;
import org.firstinspires.ftc.teamcode.SIGMA.utils.ButtonAction;
import org.firstinspires.ftc.teamcode.SIGMA.utils.PIDController;
import org.firstinspires.ftc.teamcode.SIGMA.utils.TargetedMotor;

/*
 _____ _____ _____ ___  ___  ___  
/  ___|_   _|  __ \|  \/  | / _ \ 
\ `--.  | | | |  \/| .  . |/ /_\ \
 `--. \ | | | | __ | |\/| ||  _  |
/\__/ /_| |_| |_\ \| |  | || | | |
\____/ \___/ \____/\_|  |_/\_| |_/
  u      n      e      a      l
  p      t      n      n      g
  e      e      e      a      o
  r      g      r      g      r
         r      a      e      i
         a      l      m      t
         t             e      h
         e             n      m
         d             t
*/
@TeleOp(name = "TeleopSIGMA", group = "Pushbot")
public class TeleopSIGMA extends OpMode {

    FtcDashboard dashboard;
    static final float adjustMultiplier = 0.225f;

    /* Declare OpMode members. */
    RobotHardwareSIGMA robot = new RobotHardwareSIGMA();

    double robotReferenceYaw = 0;
    double robotCurrentYaw = 0;

    boolean fieldCentricDrive_enabled = false;
    boolean slowmode = false;
    boolean inverse1 = false;
    boolean inverse2 = false;

    // Debounce Stuff - by Teo
    // TODO: Make this actually use debounce timers
    private final ButtonAction[] buttonActions = {
            // TODO: Add the buttons for field-centric-drive recentering
    };

    // Another cool functional programming interface
    // This time for the common pattern of targeted motors

    @SuppressLint("DefaultLocale")
    private final TargetedMotor[] targetedMotors = {};

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Hello thomas");

        robotReferenceYaw = robot.getOrientation().getYaw(AngleUnit.RADIANS);
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
    }

    public void wheels() {
        double joystick_y = 0.0;
        double joystick_x = 0.0;
        double joystick_yaw = 0.0;
        double joystickMultiplier = !gamepad1.right_bumper ? 0.8 : 0.45;

        joystick_y += (gamepad1.left_stick_y * joystickMultiplier) - (gamepad2.left_stick_y * adjustMultiplier);
        joystick_x += (gamepad1.left_stick_x * joystickMultiplier) - (gamepad2.left_stick_x * adjustMultiplier);
        joystick_yaw += (gamepad1.right_stick_x * joystickMultiplier) + (gamepad2.right_stick_x * adjustMultiplier);

        double strafe;
        double throttle;

        if (fieldCentricDrive_enabled) {
            // Only update yaw rotation if the robot stationary or spinning
            // This helps prevent wobbly motion when trying to move in straight lines
            if ((joystick_x == 0.0 && joystick_y == 0.0) || joystick_yaw != 0.0)
                robotCurrentYaw = robot.getOrientation().getYaw(AngleUnit.RADIANS);

            double theta = -(robotReferenceYaw - robotCurrentYaw);
            double sinT = Math.sin(theta);
            double cosT = Math.cos(theta);

            strafe = joystick_x * cosT - joystick_y * sinT;
            throttle = joystick_x * sinT + joystick_y * cosT;
        } else {
            strafe = joystick_x;
            throttle = joystick_y;
        }

        robot.lfDrive.setPower(throttle - strafe - joystick_yaw);
        robot.lbDrive.setPower(throttle + strafe - joystick_yaw);
        robot.rfDrive.setPower(-throttle - strafe - joystick_yaw);
        robot.rbDrive.setPower(-throttle + strafe - joystick_yaw);
    }

    private double launcherTargetSpeed = 0;
    private final double launcherRampSpeed = 1;

    private final double launcherMaxSpeed = 100;
    private final double launcherMinSpeed = 0;

    // TODO: Tune ts
    private PIDController launcherPID = new PIDController(1.0, 0.0, 0.0);
    private final ElapsedTime frameTimer = new ElapsedTime();

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Final Robot Instructions
        wheels();
        launcher();

        ButtonAction.doActions(buttonActions);
        TargetedMotor.runArray(targetedMotors);

        telemetry.update();
        frameTimer.reset();
    }

    private void launcher() {
        // HACK: VERY JANK launcher speed control
        launcherTargetSpeed += ((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0)) * launcherRampSpeed;
        if (launcherTargetSpeed < launcherMinSpeed) launcherTargetSpeed = launcherMinSpeed;
        else if (launcherTargetSpeed > launcherMaxSpeed) launcherTargetSpeed = launcherMaxSpeed;

        double realSpeed = robot.launcher.getVelocity();
        double control = launcherPID.update(launcherTargetSpeed, realSpeed, frameTimer.seconds());
        robot.launcher.setPower(control);

        /* Dashboard */
        telemetry.addData("launcherVel", realSpeed);
        telemetry.addData("launcherTargetVel", launcherTargetSpeed);
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
