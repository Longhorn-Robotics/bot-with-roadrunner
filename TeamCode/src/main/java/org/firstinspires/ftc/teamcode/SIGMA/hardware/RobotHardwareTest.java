package org.firstinspires.ftc.teamcode.SIGMA.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardwareTest {
    HardwareMap hwMap;
    public DcMotorEx motor;

    public RobotHardwareTest() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        hwMap = ahwMap;

        // Initialize drive motors
        motor = hwMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setPower(0);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
