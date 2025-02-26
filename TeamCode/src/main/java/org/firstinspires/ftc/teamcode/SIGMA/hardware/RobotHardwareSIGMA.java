package org.firstinspires.ftc.teamcode.SIGMA.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SIGMA.utils.GroupMotor;

public class RobotHardwareSIGMA {
    HardwareMap hwMap;

    public IMU imu;

    public DcMotor lbDrive;
    public DcMotor lfDrive;
    public DcMotor rbDrive;
    public DcMotor rfDrive;
    public DcMotor bucketRailL;
    public DcMotor bucketRailR;
    public Servo bucket;
    public Servo clawExtend;
    public Servo clawPivot;
    public Servo clawPinch;
    public Servo specimenGrabber;

    public GroupMotor railMotors;
    public GroupMotor wheels;

//    private ElapsedTime period = new ElapsedTime();

    public RobotHardwareSIGMA() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        hwMap = ahwMap;

        // Initialize IMU
        imu = hwMap.get(IMU.class, "imu");
        updateOrientation();

        // Initialize drive motors
        lfDrive = hwMap.get(DcMotor.class, "motorFL");
        lbDrive = hwMap.get(DcMotor.class, "motorBL");
        rfDrive = hwMap.get(DcMotor.class, "motorFR");
        rbDrive = hwMap.get(DcMotor.class, "motorBR");
        wheels = new GroupMotor(lfDrive, lbDrive, rfDrive, rbDrive);

        wheels.apply(dcMotor -> {
            dcMotor.setDirection(DcMotor.Direction.FORWARD);
            dcMotor.setPower(0);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        // Initialize linear rail motor to run with encoder
        bucketRailL = hwMap.get(DcMotor.class, "railL");
        bucketRailR = hwMap.get(DcMotor.class, "railR");
        bucketRailL.setDirection(DcMotor.Direction.FORWARD);
        bucketRailR.setDirection(DcMotor.Direction.REVERSE);

        railMotors = new GroupMotor(bucketRailL, bucketRailR);

        railMotors.apply((DcMotor motor) -> {
            motor.setTargetPosition(0);
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        });

        // Initialize intake servos
        clawExtend = hwMap.get(Servo.class, "extendEXTEND");
        clawPivot = hwMap.get(Servo.class, "pivotPIVOT");
        clawPinch = hwMap.get(Servo.class, "pinchPINCH");

        bucket = hwMap.get(Servo.class, "bucketL");

        specimenGrabber = hwMap.get(Servo.class, "specimenSPECIMEN");
    }

    public YawPitchRollAngles getOrinatation() {
        return imu.getRobotYawPitchRollAngles();
    }

    public void updateOrientation() {
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        } catch (IllegalArgumentException e) {
        }
    }
}
