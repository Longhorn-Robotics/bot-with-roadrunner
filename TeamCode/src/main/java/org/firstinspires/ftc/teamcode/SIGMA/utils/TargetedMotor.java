package org.firstinspires.ftc.teamcode.SIGMA.utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class TargetedMotor {
    double minimum;
    double maximum;
    DoubleSupplier target;
    DoubleConsumer setTarget;
    DoubleConsumer motorSetter;

    public TargetedMotor(double min, double max, DoubleSupplier targetSupplier, DoubleConsumer targetSetter, DoubleConsumer targetingMotorSetter) {
        minimum = min;
        maximum = max;
        target = targetSupplier;
        setTarget = targetSetter;
        motorSetter = targetingMotorSetter;
    }

    public void run() {
        setTarget.accept(Math.min(Math.max(target.getAsDouble(), minimum), maximum));
        motorSetter.accept(target.getAsDouble());
    }

    public static void runArray(TargetedMotor[] motors) {
        for (TargetedMotor motor : motors) {
            motor.run();
        }
    }

}
