package org.firstinspires.ftc.teamcode.SIGMA.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class RobotHardwareCamera {
    HardwareMap hwMap;
    //public

    public WebcamName cam;
    public RobotHardwareCamera() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        hwMap = ahwMap;

        cam = hwMap.get(WebcamName.class, "Webcam 1");
    }
}
