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
    double currentSpeed = -0.7;
    double targetSpeed = 4000;
    double pidf_last_error_1 = 0;
    double pidf_last_error_2 = 0;
    double increaseRate = 0.001;
    boolean isAdd = false;
    boolean isSubtract = false;
    boolean isKickerExtended = false;
    boolean x_pressed = false;

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

        if(gamepad1.right_bumper && !isAdd)
        {
            currentSpeed += 0.05;
            isAdd = true;
        }
        else if(!gamepad1.right_bumper)
        {
            isAdd = false;
        }

        if(gamepad1.left_bumper && !isSubtract)
        {
            currentSpeed -= 0.05;
            isSubtract = true;
        }
        else if(!gamepad1.left_bumper)
        {
            isSubtract = false;
        }
        currentSpeed = Math.min(currentSpeed, 1);
        currentSpeed = Math.max(currentSpeed, -1);

//        targetSpeed = 6000 * currentSpeed;

        //robot.intakeMotor.setVelocity(currentSpeed * 360 / 60, AngleUnit.DEGREES);
        robot.intakeMotor.setPower(currentSpeed);

//        double trueSpeed, error, output;

//        // PIDF coeffs
//        double feedforward_coeff = targetSpeed / 6000.0;
//        double proportional_coeff = 0.01;
//        double derivative_coeff = 0.01;

        telemetry.addData("Current Speed: ", currentSpeed);
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

        /*
        if(gamepad1.x && !x_pressed)
        {
            isKickerExtended = !isKickerExtended;
            runtime.reset();
        } else if (runtime.seconds() > 1) {
            isKickerExtended = false;
        }
        x_pressed = gamepad1.x;

        if(isKickerExtended)
        {
            robot.kicker.setPosition(0.07); //0.55
        }
        else if(!isKickerExtended)
        {
            robot.kicker.setPosition(0.247); //0.73
        }

        telemetry.addData("Is Kicker Extended? ", isKickerExtended);
        telemetry.addData("Runtime: ", runtime);'=
         */

        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}