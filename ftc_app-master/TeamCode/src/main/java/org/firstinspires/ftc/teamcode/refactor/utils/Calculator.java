package org.firstinspires.ftc.teamcode.refactor.utils;

import org.firstinspires.ftc.teamcode.refactor.utils.Constants;

public class Calculator {
    public static int getTicks(double inches) {
        return (int) Math.round((inches / Constants.WHEEL_CIRCUMFERENCE * Constants.TICKS_PER_REV * Constants.GEAR_RATIO));
    }
    public static double getInches(int ticks) {
        return (ticks / Constants.TICKS_PER_REV * Constants.WHEEL_CIRCUMFERENCE * (1 / Constants.GEAR_RATIO));
    }
}
