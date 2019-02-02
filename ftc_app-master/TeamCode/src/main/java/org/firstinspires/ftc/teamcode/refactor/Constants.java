package org.firstinspires.ftc.teamcode.refactor;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Constants {
    //physical constants of the robot
    public static final double WHEEL_RADIUS = 4.0;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;
    public static final double GEAR_RATIO = 2.0;
    public static final double LIFT_GEAR_RATIO = 0.0;

    //drive constants
    public static final double TICKS_PER_REV = MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class).getTicksPerRev();
    public static final double DRIVE_SPEED = 0.78;
    public static final double TURN_SPEED = 0.57;
    public static final double LIFT_SPEED = 1.0;

    //lift constants:
    //maximum and minimum positions of the robot's lift during autonomous mode
    public static final int AUTO_MAX_LIFT_TICKS = -7497;
    public static final int AUTO_MIN_LIFT_TICKS = 0;

    //maximum and minimum positions of the robot's lift during tele-operated mode
    public static final int MAX_LIFT_TICKS = -10516;
    public static final int MIN_LIFT_TICKS = 0;

    //pid information
    public static final PIDCoefficients PID_DRIVE_COEFFICIENTS = new PIDCoefficients(0.085, 0.0, 0.42);
    public static final PIDCoefficients PID_TURN_COEFFICIENTS = new PIDCoefficients(0.1, 0.0, 0.42);
    public static final PIDCoefficients PID_STRAFE_COEFFICIENTS = new PIDCoefficients(0.1, 0.0, 0.42);
    public static final double HEADING_THRESHOLD = 1.5;
}
