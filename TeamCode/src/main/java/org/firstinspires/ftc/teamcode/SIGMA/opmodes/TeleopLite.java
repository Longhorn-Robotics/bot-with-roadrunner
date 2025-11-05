package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SIGMA.hardware.RobotHardwareLite;

@TeleOp(name = "TeleopLite", group = "Pushbot")
public class TeleopLite extends OpMode {
    RobotHardwareLite robot = new RobotHardwareLite();

    //YOUSEF TEST
    double currentElevatorSpeed = 0.7;
    double currentIntakeSpeed = 0.7;
    double currentGunSpeed = 0.7;
    double targetSpeed = 4000;
    double pidf_last_error_1 = 0;
    double pidf_last_error_2 = 0;
    double increaseRate = 0.001;
    boolean isIntakeAdd = false;
    boolean isIntakeSubtract = false;

    boolean isElevatorAdd = false;
    boolean isElevatorSubtract = false;

    boolean isGunAdd = false;
    boolean isGunSubtract = false;

    boolean isKickerExtended = false;
    boolean x_pressed = false;

    boolean isFlickerExtended = false;
    boolean square_pressed = false;

    double intakeTargetSpeed = 20.0;

    //YOUSEF TIME
    private ElapsedTime runtime = new ElapsedTime();

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
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        /*intakeTargetSpeed += gamepad1.right_stick_x * 1.0;
        robot.intakeMotor.setVelocity(20.0, AngleUnit.DEGREES);
        currentSpeed -= gamepad1.left_stick_y * increaseRate;*/

        //Gun Motor
        if(gamepad1.dpad_up && !isGunAdd)
        {
            currentGunSpeed += 0.05;
            isGunAdd = true;
        }
        else if(!gamepad1.dpad_up)
        {
            isGunAdd = false;
        }

        if(gamepad1.dpad_down && !isGunSubtract)
        {
            currentGunSpeed -= 0.05;
            isGunSubtract = true;
        }
        else if(!gamepad1.dpad_down)
        {
            isGunSubtract = false;
        }
        currentGunSpeed = Math.min(currentGunSpeed, 1);
        currentGunSpeed = Math.max(currentGunSpeed, 0);

        //Intake Motor
        if(gamepad1.right_bumper && !isIntakeAdd)
        {
            currentIntakeSpeed += 0.05;
            isIntakeAdd = true;
        }
        else if(!gamepad1.right_bumper)
        {
            isIntakeAdd = false;
        }

        if(gamepad1.left_bumper && !isIntakeSubtract)
        {
            currentIntakeSpeed -= 0.05;
            isIntakeSubtract = true;
        }
        else if(!gamepad1.left_bumper)
        {
            isIntakeSubtract = false;
        }
        currentIntakeSpeed = Math.min(currentIntakeSpeed, 1);
        currentIntakeSpeed = Math.max(currentIntakeSpeed, 0);

        //Elevator Motor
        if(gamepad1.right_trigger > 0.5 && !isElevatorAdd)
        {
            currentElevatorSpeed += 0.001;
            isElevatorAdd = true;
        }
        else
        {
            isElevatorAdd = false;
        }

        if(gamepad1.left_trigger > 0.5 && !isElevatorSubtract)
        {
            currentElevatorSpeed -= 0.001;
            isElevatorSubtract = true;
        }
        else
        {
            isElevatorSubtract = false;
        }
        currentElevatorSpeed = Math.min(currentElevatorSpeed, 1);
        currentElevatorSpeed = Math.max(currentElevatorSpeed, 0);

//        targetSpeed = 6000 * currentSpeed;

        //robot.intakeMotor.setVelocity(currentSpeed * 360 / 60, AngleUnit.DEGREES);

        //Set Powers
        robot.elevatorMotor.setPower(-currentElevatorSpeed);
        robot.intakeMotor.setPower(currentIntakeSpeed);

        robot.motorFlywheel1.setPower(currentGunSpeed);
        robot.motorFlywheel2.setPower(-currentGunSpeed);

        //DATA
        telemetry.addData("Current Intake Speed: ", currentIntakeSpeed);
        telemetry.addData("Current Elevator Speed: ", currentElevatorSpeed);
        telemetry.addData("Current Gun Speed: ", currentGunSpeed);

//        double trueSpeed, error, output;

//        // PIDF coeffs
//        double feedforward_coeff = targetSpeed / 6000.0;
//        double proportional_coeff = 0.01;
//        double derivative_coeff = 0.01;

        //telemetry.addData("Target Speed: ", targetSpeed);
        // Motor 1
        // Converting degrees/sec to rpm
        /*trueSpeed = robot.motorFlywheel1.getVelocity(AngleUnit.DEGREES) / 6.0;
        error = targetSpeed - trueSpeed;
        output = error * proportional_coeff + (error - pidf_last_error_1) * derivative_coeff + feedforward_coeff;
        pidf_last_error_1 = error;
        robot.motorFlywheel1.setPower(output);
        telemetry.addData("Motor 1 Speed: ", trueSpeed);

        trueSpeed = robot.motorFlywheel2.getVelocity(AngleUnit.DEGREES) / 6.0;
        error = targetSpeed - trueSpeed;
        output = error * proportional_coeff + (error - pidf_last_error_2) * derivative_coeff + feedforward_coeff;
        pidf_last_error_2 = error;
        robot.motorFlywheel2.setPower(-output);
        telemetry.addData("Motor 2 Speed: ", trueSpeed);*/

//        trueSpeed = robot.intakeMotor.getVelocity(AngleUnit.DEGREES) / 6.0;
//        error = targetSpeed - trueSpeed;
//        output = error * proportional_coeff + (error - pidf_last_error_1) * derivative_coeff + feedforward_coeff;
//        pidf_last_error_1 = error;
//        robot.intakeMotor.setPower(output);
//        telemetry.addData("Motor 1 Speed: ", trueSpeed);

        //KICKER
        if(gamepad1.cross && !x_pressed)
        {
            isKickerExtended = !isKickerExtended;
            runtime.reset();
        } else if (runtime.seconds() > 1) {
            isKickerExtended = false;
        }
        x_pressed = gamepad1.cross;

        if(isKickerExtended)
        {
            robot.kicker.setPosition(0.07); //0.55
        }
        else if(!isKickerExtended)
        {
            robot.kicker.setPosition(0.247); //0.73
        }

        //FLICKER
        if(gamepad1.square && !square_pressed)
        {
            isFlickerExtended = !isFlickerExtended;
            runtime.reset();
        } else if (runtime.seconds() > 0.25) {
            isFlickerExtended = false;
        }
        square_pressed = gamepad1.square; //MERCURIO FOR TEO

        if(isFlickerExtended)
        {
            robot.flicker.setPosition(0.35);
        }
        else if(!isFlickerExtended)
        {
            robot.flicker.setPosition(0.17);
        }

        telemetry.addData("Flicker", isFlickerExtended);
        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}