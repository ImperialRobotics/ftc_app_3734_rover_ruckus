package org.firstinspires.ftc.teamcode.refactor.utils;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Constants {
    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    public static final double WHEEL_RADIUS = 2.0;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;
    public static final double GEAR_RATIO = 2.0;
    public static final double LIFT_GEAR_RATIO = 0.0;

    //----------------------------------------------------------------------------------------------
    // DriveConstants
    //----------------------------------------------------------------------------------------------

    public static final double TICKS_PER_REV = MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class).getTicksPerRev();
    public static final double DRIVE_SPEED = 0.88;

    //----------------------------------------------------------------------------------------------
    // Lift Constants
    //----------------------------------------------------------------------------------------------

    public static final int AUTO_MAX_LIFT_TICKS = -7497;
    public static final int MAX_LIFT_TICKS = -10516;
    public static final double LIFT_SPEED = 1.0;

    //----------------------------------------------------------------------------------------------
    // Intake Constants
    //----------------------------------------------------------------------------------------------

    //speeds
    public static final double EXTEND_SPEED = 1.0;
    public static final double ROTATE_SPEED = 1.0;
    public static final double INTAKE_SPEED = 1.0;

    //maximum limit ticks
    public static final double INTAKE_TICKS_PER_REV = MotorConfigurationType.getMotorType(RevRoboticsCoreHexMotor.class).getTicksPerRev();
    public static final int MAX_EXTEND_TICKS = 0;
    public static final double MAX_ROTATE_TICKS = 0.0;

    //----------------------------------------------------------------------------------------------
    // Pid Information
    //----------------------------------------------------------------------------------------------

    public static final PIDCoefficients PID_DRIVE_COEFFICIENTS = new PIDCoefficients(0.085, 0.0, 0.42);
    public static final PIDCoefficients PID_TURN_COEFFICIENTS = new PIDCoefficients(0.1, 0.0, 0.42);
    public static final PIDCoefficients PID_STRAFE_COEFFICIENTS = new PIDCoefficients(0.1, 0.0, 0.42);
    public static final double HEADING_THRESHOLD = 1.5;
}
