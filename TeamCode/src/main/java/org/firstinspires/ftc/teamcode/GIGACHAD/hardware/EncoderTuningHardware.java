package org.firstinspires.ftc.teamcode.GIGACHAD.hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class EncoderTuningHardware {
    HardwareMap hwMap;

    public DcMotorEx encoderLeft;
    public DcMotorEx encoderRight;
    public DcMotorEx encoderPerp;

    public EncoderTuningHardware() {}

    public void init(HardwareMap ahwMap) {
        encoderLeft = setupMotor(ahwMap, "");
//        encoderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderRight = setupMotor(ahwMap, "");
        encoderPerp = setupMotor(ahwMap, "");
    }

    private DcMotorEx setupMotor(HardwareMap ahwMap, String hwmap_name) {
        DcMotorEx deadWheelMotor = ahwMap.get(DcMotorEx.class, hwmap_name);
        deadWheelMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        return deadWheelMotor;
    }
}
