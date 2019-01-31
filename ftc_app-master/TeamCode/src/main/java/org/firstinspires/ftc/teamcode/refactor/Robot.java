package org.firstinspires.ftc.teamcode.refactor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {
    //motors and servos
    public DcMotor leftMotor, rightMotor, strafeMotor, armMotor;
    public DcMotor[] motors;
    public Servo markerServo;

    //imu, used for calculating the heading of the robot
    public BNO055IMU imu;

    //opMode used for robot construction
    public LinearOpMode opMode;

    //PID controller, used for driving and turning
    private PIDController pidController;

    //constants for driving and control
    public static final double DRIVE_SPEED = 0.78;
    public static final double TURN_SPEED = 0.57;
    public static final PIDCoefficients PID_DRIVE_COEFFICIENTS = new PIDCoefficients(0.1, 0.0, 0.42);
    public static final PIDCoefficients PID_TURN_COEFFICIENTS = new PIDCoefficients(0.1, 0.0, 0.42);
    public static final double HEADING_THRESHOLD = 1.5;

    public Robot(LinearOpMode opMode) {
        //initializing motors from hardware map
        leftMotor = opMode.hardwareMap.dcMotor.get("motorLeft");
        rightMotor = opMode.hardwareMap.dcMotor.get("motorLeft");
        strafeMotor = opMode.hardwareMap.dcMotor.get("motorLeft");
        armMotor = opMode.hardwareMap.dcMotor.get("motorArm");
        markerServo = opMode.hardwareMap.servo.get("servoMarker");

        //configuring motors to their appropriate direction
        motors = new DcMotor[] {leftMotor, rightMotor, strafeMotor, armMotor};
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //resetting encoder counts and run modes of motors
        for(DcMotor motor: motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        pidController = new PIDController(-1.0, 1.0);
        this.opMode = opMode;
    }

    //method used in autonomous programs to configure the imu appropriate
    public void initAutonomous() {
        //parameters used for the initialization of the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        //initializing imu with the parameters above and using the hardware map
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //updating telemetry appropriately to signal the calibration of the imu
        opMode.telemetry.addData("Status", "calibrating imu...");
        opMode.telemetry.update();

        //make sure the imu gyro is calibrated before continuing
        while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
            opMode.sleep(50);
            opMode.idle();
        }
    }

    //method used for displaying telemetry information during the tele-operated modes
    public void telemetry() {
        opMode.telemetry.addData("Motors", "left: (%.2f), right: (%.2f), strafe: (%.2f), arm: (%.2f)", leftMotor.getPower(), rightMotor.getPower(), strafeMotor.getPower(), armMotor.getPower());
        opMode.telemetry.addData("Heading", getHeading());
        opMode.telemetry.update();
    }

    //method used for displaying telemetry information during the autonomous period
    public void autonomousTelemetry() {
        opMode.telemetry.addData("Motors", "left: (%.2f), right: (%.2f), strafe: (%.2f), arm: (%.2f)", leftMotor.getPower(), rightMotor.getPower(), strafeMotor.getPower(), armMotor.getPower());
        opMode.telemetry.addData("Encoders", "left: (%d), right: (%d), strafe: (%d), arm: (%d)", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition(), strafeMotor.getCurrentPosition(), armMotor.getCurrentPosition());
        opMode.telemetry.addData("Targets", "left: (%d), right: (%d), strafe: (%d), arm: (%d)", leftMotor.getTargetPosition(), rightMotor.getTargetPosition(), strafeMotor.getTargetPosition(), armMotor.getTargetPosition());
        opMode.telemetry.update();
    }

    //methods to help switch between encoder on and off states
    public void withEncoder() {
        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void withoutEncoder() {
        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    //methods to help make setting powers easier and more efficient

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void setPower(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
    public void setPower(double leftPower, double rightPower, double strafePower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        strafeMotor.setPower(strafePower);
    }

    //methods to help make setting target position easier and more efficient
    public void setTargetPosition(double target) {
        withEncoder();
        leftMotor.setTargetPosition(Calculator.getTicks(target));
        rightMotor.setTargetPosition(Calculator.getTicks(target));
    }
    public void setTargetPosition(double leftTarget, double rightTarget) {
        withEncoder();
        leftMotor.setTargetPosition(Calculator.getTicks(leftTarget));
        rightMotor.setTargetPosition(Calculator.getTicks(rightTarget));
    }
    public void setTargetPosition(double leftTarget, double rightTarget, double strafeTarget) {
        withEncoder();
        leftMotor.setTargetPosition(Calculator.getTicks(leftTarget));
        rightMotor.setTargetPosition(Calculator.getTicks(rightTarget));
        strafeMotor.setTargetPosition(Calculator.getTicks(strafeTarget));
    }

    //method to get heading of robot, used for rotation and driving in a straight line
    public double getHeading() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;

        return angle;
    }

    public void drive(double inches) {
        drive(inches, getHeading());
    }

    public void drive(double inches, double angle) {
        int moveTicks = Calculator.getTicks(inches);
        setTargetPosition(inches);
        setPower(DRIVE_SPEED);
        pidController.reset();
        pidController.pidCoefficients = PID_DRIVE_COEFFICIENTS;
        pidController.targetPosition = angle;

        while(leftMotor.isBusy() && rightMotor.isBusy()) {
            double currentAngle = getHeading();
            double steer = pidController.update(currentAngle);

            if (inches < 0)
                steer *= -1.0;

            double leftSpeed = DRIVE_SPEED - steer;
            double rightSpeed = DRIVE_SPEED + steer;

            setPower(leftSpeed, rightSpeed);
        }
        setPower(0);
        withoutEncoder();
    }
}
