package org.firstinspires.ftc.teamcode.SIGMA.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardwareTest {
    HardwareMap hwMap;
//    public DcMotor lbDrive;
//    public DcMotor lfDrive;
//    public DcMotor rbDrive;
//    public DcMotor rfDrive;
//    public DcMotor hangDrive;
    public DcMotor railL;
    public DcMotor railR;
//    public DcMotor arm;
//    public Servo claw;

//    private ElapsedTime period = new ElapsedTime();

    public RobotHardwareTest() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        hwMap = ahwMap;
//
//        // Initialize drive motors
//        lfDrive = hwMap.get(DcMotor.class, "motorFL");
//        lbDrive = hwMap.get(DcMotor.class, "motorBL");
//        rfDrive = hwMap.get(DcMotor.class, "motorFR");
//        rbDrive = hwMap.get(DcMotor.class, "motorBR");
//        lfDrive.setDirection(DcMotor.Direction.FORWARD);
//        lbDrive.setDirection(DcMotor.Direction.FORWARD);
//        rfDrive.setDirection(DcMotor.Direction.FORWARD);
//        rbDrive.setDirection(DcMotor.Direction.FORWARD);
//        lfDrive.setPower(0);
//        lbDrive.setPower(0);
//        rfDrive.setPower(0);
//        rbDrive.setPower(0);
//        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        hangDrive = hwMap.get(DcMotor.class, "hangHANG");
//        hangDrive.setDirection(DcMotor.Direction.FORWARD);
//        hangDrive.setPower(0);
//        hangDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hangDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize linear rail motor to run with encoder
        railL = hwMap.get(DcMotor.class, "railL");
        railL.setDirection(DcMotor.Direction.FORWARD);
        railL.setTargetPosition(0);
        railL.setPower(0.8);
        railL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        railL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        railL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        railL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Initialize linear rail motor to run with encoder
        railR = hwMap.get(DcMotor.class, "railR");
        railR.setDirection(DcMotor.Direction.REVERSE);
        railR.setTargetPosition(0);
        railR.setPower(0.8);
        railR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        railR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        railR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        railR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        // Initialize arm motor to run with encoder
//        arm = hwMap.get(DcMotor.class, "armARM");
//        arm.setDirection(DcMotor.Direction.FORWARD);
//        arm.setTargetPosition(0);
//        arm.setPower(0.85);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Initialize claw servos
//        claw = hwMap.get(Servo.class, "clawCLAW");
    }
}
