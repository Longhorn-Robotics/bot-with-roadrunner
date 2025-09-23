package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTest")
public class MotorTest extends OpMode {
    public DcMotor TestMotor1;
    public DcMotor TestMotor2;

    @Override
    public void init() {

        TestMotor1 = hardwareMap.get(DcMotor.class, "launchMotor");
        TestMotor2 = hardwareMap.get(DcMotor.class, "launchMotor2");
    }@Override
    public void start() {

        TestMotor1.setPower(1);
        TestMotor2.setPower(-1);
    }
    @Override
    public void init_loop() {

    }
    @Override
    public void loop() {

    }@Override
    public void stop() {

    }
}
