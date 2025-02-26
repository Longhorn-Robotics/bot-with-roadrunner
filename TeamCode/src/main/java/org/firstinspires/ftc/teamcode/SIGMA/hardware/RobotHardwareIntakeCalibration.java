package org.firstinspires.ftc.teamcode.SIGMA.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardwareIntakeCalibration {
    HardwareMap hwMap;

    public Servo pincher;
//    public Servo extender;
//    public Servo pivot;
//    public Servo bucketL;
//    public Servo bucketR;
//    public Servo specimenGrabber;

    private ElapsedTime period = new ElapsedTime();

    public RobotHardwareIntakeCalibration() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        hwMap = ahwMap;

        pincher = hwMap.get(Servo.class, "pinchPINCH");
//        extender = hwMap.get(Servo.class, "extendEXTEND");
//        pivot = hwMap.get(Servo.class, "pivotPIVOT");
//        bucketL = hwMap.get(Servo.class, "bucketL");
//        bucketR = hwMap.get(Servo.class, "bucketR");

//        specimenGrabber = hwMap.get(Servo.class, "specimenSPECIMEN");
    }
}
