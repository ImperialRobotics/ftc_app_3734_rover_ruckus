package org.firstinspires.ftc.teamcode.refactor;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Constants {
    public static final double WHEEL_RADIUS = 4.0;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;

    public static final double GEAR_RATIO = 2.0;

    public static final double TICKS_PER_REV = MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class).getTicksPerRev();
}
