package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SIGMA.hardware.RobotHardwareSIGMA;
import org.firstinspires.ftc.teamcode.SIGMA.utils.ButtonAction;
import org.firstinspires.ftc.teamcode.SIGMA.utils.TargetedMotor;
import org.firstinspires.ftc.teamcode.SIGMA.utils.Utils;

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

    static final float adjustMultiplier = 0.225f;

    /* Declare OpMode members. */
    RobotHardwareSIGMA robot = new RobotHardwareSIGMA();

    double robotReferenceYaw = 0;
    double robotCurrentYaw = 0;

    boolean ftc_enabled = false;
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
    private final TargetedMotor[] targetedMotors = {

//            new TargetedMotor(RAIL_MIN, RAIL_MAX, () -> railPosition, x -> railPosition = x, a -> {
//                robot.railMotors.apply(motor -> {
////                motor.setTargetPosition((int) railPosition);
//                    if (motor.getCurrentPosition() > railPosition) motor.setPower(0.4);
//                    else motor.setPower(0.8);
//                });
//                robot.bucketRailL.setTargetPosition((int) railPosition);
//                robot.bucketRailR.setTargetPosition((int) railPosition + 20);
//            }),
//            new TargetedMotor(EXTEND_OUT, EXTEND_IN, () -> extendPosition, a -> extendPosition = a, a -> robot.clawExtend.setPosition(a))
    };

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
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

        if (ftc_enabled) {
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

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Final Robot Instructions
        wheels();

        ButtonAction.doActions(buttonActions);
        TargetedMotor.runArray(targetedMotors);

        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
