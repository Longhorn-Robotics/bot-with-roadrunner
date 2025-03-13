package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    static final double RAIL_MIN = 0.0;
    static final double RAIL_MAX = 2800.0;
    static final double PINCH_OPEN = 0.66;
    static final double PINCH_CLOSED = 0.89;
    static final double EXTEND_IN = 0.42;
    static final double EXTEND_OUT = 0.12;
    static final double PIVOT_DOWN = 0.04;
    static final double PIVOT_FLOAT = 0.25;
    static final double PIVOT_BACK = 0.90;
    // Tipped: 0.82
    // Hold: 0.68
    // Load: 0.54
    static final double BUCKET_DUMP = 0.82;
    static final double BUCKET_HOLD = 0.68;
    static final double BUCKET_PICK = 0.54;
    static final float adjustMultiplier = 0.225f;
    static final double[] bucketPositions = new double[]{BUCKET_PICK, BUCKET_HOLD, BUCKET_DUMP};
    static final double SPECIMEN_CLOSE = 0.068;
    static final double SPECIMEN_OPEN = 0.452;

    /* Declare OpMode members. */
    RobotHardwareSIGMA robot = new RobotHardwareSIGMA();

    double robotReferenceYaw = 0;
    double robotCurrentYaw = 0;

    // Only allows the rail to be moved if the claw is out of the way and vice versa
    enum ArticulationFocus {
        CLAW, BUCKET, NONE
    }
    ArticulationFocus focus = ArticulationFocus.NONE;
    private boolean focusQualifier(ArticulationFocus focusGroup) {
        if (focus == focusGroup) return true;
        else if (focus == ArticulationFocus.NONE) {
            focus = focusGroup;
            return true;
        }
        return false;
    }

    double railPosition = RAIL_MIN;
    double extendPosition = EXTEND_IN;
    double pinchPosition = PINCH_OPEN;

    int pivotState = 2;
    int bucketState = 0;
    boolean specimenGrabbing = true;

    boolean slowmode = false;
    boolean inverse1 = false;
    boolean inverse2 = false;

    // Debounce Stuff - by Teo
    // TODO: Find better acronyms
    private final ButtonAction[] buttonActions = {
//            new ButtonAction(() -> gamepad1.right_bumper, () -> slowmode = !slowmode),
            new ButtonAction(() -> gamepad1.left_bumper, () -> inverse1 = !inverse1),
            new ButtonAction(() -> gamepad2.left_bumper, () -> inverse2 = !inverse2),
            new ButtonAction(() -> gamepad2.right_bumper, () -> {
                if (extendPosition < ((EXTEND_OUT + EXTEND_IN) * 0.5) ) {
                    extendPosition = EXTEND_IN;
                    pivotState = 2;
                } else {
                    extendPosition = EXTEND_IN + (EXTEND_OUT - EXTEND_IN) * 0.75;
                    pivotState = 1;
                    focus = ArticulationFocus.NONE;
                }
            }),
            // Toggle between float and down
            new ButtonAction(() -> (gamepad2.triangle && focusQualifier(ArticulationFocus.CLAW)), () -> pivotState = (new int[]{1, 0, 0})[pivotState]),
            new ButtonAction(() -> (gamepad2.dpad_left && focusQualifier(ArticulationFocus.CLAW)), () -> pivotState++),
            new ButtonAction(() -> (gamepad2.dpad_right && focusQualifier(ArticulationFocus.CLAW)), () -> pivotState--),
            new ButtonAction(() -> (gamepad2.circle && focusQualifier(ArticulationFocus.CLAW)), () -> {
                // Protects from smashing the inside of the bucket if some bum forgets about the impeccable pull out game
                if ((extendPosition - EXTEND_IN) / (EXTEND_OUT - EXTEND_IN) < 0.2) return;
                if (pinchPosition == PINCH_CLOSED) pinchPosition = PINCH_OPEN;
                else pinchPosition = PINCH_CLOSED;
            }),
            new ButtonAction(() -> gamepad1.y, () -> specimenGrabbing = !specimenGrabbing),
            new ButtonAction(() -> (gamepad2.cross && focusQualifier(ArticulationFocus.CLAW)), () -> {
                pinchPosition = (PINCH_CLOSED + PINCH_OPEN) * 0.5;
                Utils.setTimeout(200, () -> {
                    extendPosition -= 0.04;
                    Utils.setTimeout(300, () -> {
                        pivotState = 1;
                        bucketState = 1;
                        focus = ArticulationFocus.BUCKET;
                    });
                });
            }),
            new ButtonAction(() -> (gamepad1.square && focusQualifier(ArticulationFocus.BUCKET)), () -> bucketState = (new int[]{2, 2, 0})[bucketState])
    };

    // Another cool functional programming interface
    // This time for the common pattern of targeted motors

    @SuppressLint("DefaultLocale")
    private final TargetedMotor[] targetedMotors = {
            new TargetedMotor(RAIL_MIN, RAIL_MAX, () -> railPosition, x -> railPosition = x, a -> {
                robot.railMotors.apply(motor -> {
//                motor.setTargetPosition((int) railPosition);
                    if (motor.getCurrentPosition() > railPosition) motor.setPower(0.4);
                    else motor.setPower(0.8);
                });
                robot.bucketRailL.setTargetPosition((int) railPosition);
                robot.bucketRailR.setTargetPosition((int) railPosition + 20);
            }),
//            new TargetedMotor(EXTEND_OUT, EXTEND_IN, () -> extendPosition, a -> extendPosition = a, a -> robot.clawExtend.setPosition(a))
    };

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Hello thomas");

        robot.railMotors.apply((dcMotor -> {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dcMotor.setTargetPosition((int) RAIL_MIN);
        }));

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

        // Only update yaw rotation if the robot stationary or spinning
        // This helps prevent wobbly motion when trying to move in straight lines
        if ((joystick_x == 0.0 && joystick_y == 0.0) || joystick_yaw != 0.0) robotCurrentYaw = robot.getOrientation().getYaw(AngleUnit.RADIANS);

        double theta = -(robotReferenceYaw - robotCurrentYaw);
        double sinT = Math.sin(theta);
        double cosT = Math.cos(theta);

        double corrected_strafe = joystick_x * cosT - joystick_y * sinT;
        double corrected_throttle = joystick_x * sinT + joystick_y * cosT;

        robot.lfDrive.setPower(corrected_throttle - corrected_strafe - joystick_yaw);
        robot.lbDrive.setPower(corrected_throttle + corrected_strafe - joystick_yaw);
        robot.rfDrive.setPower(-corrected_throttle - corrected_strafe - joystick_yaw);
        robot.rbDrive.setPower(-corrected_throttle + corrected_strafe - joystick_yaw);
    }

    @SuppressLint("DefaultLocale")
    public void rail() {
        if (focusQualifier(ArticulationFocus.BUCKET)){
            // Handled by targetedMotors
            railPosition += (gamepad2.right_trigger - gamepad2.left_trigger) * 20.0f;
            if ((railPosition - RAIL_MIN)/(RAIL_MAX - RAIL_MIN) < 0.07) focus = ArticulationFocus.NONE;
            telemetry.addLine(String.format("Target RAIL Position: %d", (int) railPosition));
        }
    }

    @SuppressLint("DefaultLocale")
    public void pinch() {
//        if (gamepad1.dpad_up) pinchPosition -= 0.005;
//        if (gamepad1.dpad_down) pinchPosition += 0.005;

//        telemetry.addLine(String.format("Target CLAW Position: %f", pinchPosition));
        robot.clawPinch.setPosition(pinchPosition);
        telemetry.addLine(String.format("Current CLAW Position: %f", robot.clawPinch.getPosition()));
    }


    @SuppressLint("DefaultLocale")
    public void extend() {
        if (gamepad2.dpad_up) extendPosition -= 0.002;
        if (gamepad2.dpad_down) extendPosition += 0.002;

        extendPosition = Math.min(Math.max(extendPosition, EXTEND_OUT), EXTEND_IN);
        robot.clawExtend.setPosition(extendPosition);

        telemetry.addLine(String.format("Target EXTEND Position: %f", extendPosition));
        telemetry.addLine(String.format("Real EXTEND Position: %f", robot.clawExtend.getPosition()));
    }

    @SuppressLint("DefaultLocale")
    public void pivot() {
        pivotState = Math.min(Math.max(pivotState, 0), 2);

        robot.clawPivot.setPosition(new double[]{PIVOT_DOWN, PIVOT_FLOAT, PIVOT_BACK}[pivotState]);
    }

    @SuppressLint("DefaultLocale")
    public void bucket() {
        robot.bucket.setPosition(bucketPositions[bucketState]);

//        if (gamepad1.dpad_up) bucketPosition += 0.02;
//        if (gamepad1.dpad_down) bucketPosition -= 0.02;

//        robot.bucket.setPosition(bucketPosition);
        telemetry.addLine(String.format("Real Bucket Position: %f", robot.bucket.getPosition()));
    }


    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Final Robot Instructions
        wheels();
        rail();
        bucket();
        extend();
        pinch();
        pivot();

        robot.specimenGrabber.setPosition(specimenGrabbing ? SPECIMEN_CLOSE : SPECIMEN_OPEN);

        ButtonAction.doActions(buttonActions);
        TargetedMotor.runArray(targetedMotors);

        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
