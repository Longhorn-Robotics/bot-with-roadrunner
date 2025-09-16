package org.firstinspires.ftc.teamcode.SIGMA.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp(name="MotorTest")
public class MotorTest extends OpMode {
    public DcMotor TestMotor;

    @Override
    public void init() {
        TestMotor = hardwareMap.get(DcMotor.class, "launchMotor");
    }@Override
    public void start() {
        TestMotor.setPower(1);
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
