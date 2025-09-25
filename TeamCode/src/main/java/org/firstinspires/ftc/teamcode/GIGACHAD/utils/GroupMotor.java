package org.firstinspires.ftc.teamcode.GIGACHAD.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.Consumer;

public class GroupMotor {
    DcMotorEx[] motors;

    public GroupMotor(DcMotorEx... _motors) {
        motors = _motors;
    }

    public void apply(Consumer<DcMotor> function) {
        for (DcMotorEx motor : motors)
            function.accept(motor);
    }
}
