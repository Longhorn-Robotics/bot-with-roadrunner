package org.firstinspires.ftc.teamcode.GIGACHAD.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GIGACHAD.utils.GroupMotor;

public class RobotHardwareGIGACHAD {
    HardwareMap hwMap;

    public IMU imu;

    /* Wheels */

    public GroupMotor wheels;
    public DcMotorEx lbDrive;
    public DcMotorEx lfDrive;
    public DcMotorEx rbDrive;
    public DcMotorEx rfDrive;


    public DcMotorEx launcher;

//    private ElapsedTime period = new ElapsedTime();

    public RobotHardwareGIGACHAD() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        hwMap = ahwMap;

        // Initialize IMU
        imu = hwMap.get(IMU.class, "imu");
        updateOrientation();

        // Initialize drive motors
        lfDrive = hwMap.get(DcMotorEx.class, "motorFL");
        lbDrive = hwMap.get(DcMotorEx.class, "motorBL");
        rfDrive = hwMap.get(DcMotorEx.class, "motorFR");
        rbDrive = hwMap.get(DcMotorEx.class, "motorBR");
        wheels = new GroupMotor(lfDrive, lbDrive, rfDrive, rbDrive);

        wheels.apply(dcMotor -> {
            dcMotor.setDirection(DcMotor.Direction.FORWARD);
            dcMotor.setPower(0);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public YawPitchRollAngles getOrientation() {
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
