package org.firstinspires.ftc.teamcode.refactor;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {
    //motors and servos
    public DcMotor leftMotor, rightMotor, strafeMotor, liftMotor;
    public DcMotor[] motors;
    public Servo markerServo;

    //imu, used for calculating the heading of the robot
    public BNO055IMU imu;

    //opMode used for robot construction
    public LinearOpMode opMode;

    //PID controller, used for driving and turning
    private PIDController pidController;

    //GoldMineralDetector for sampling
    public GoldMineralDetector detector;


    public Robot(LinearOpMode opMode) {
        //initializing motors from hardware map
        leftMotor = opMode.hardwareMap.dcMotor.get("motorLeft");
        rightMotor = opMode.hardwareMap.dcMotor.get("motorRight");
        strafeMotor = opMode.hardwareMap.dcMotor.get("motorMiddle");
        liftMotor = opMode.hardwareMap.dcMotor.get("motorArm");
        markerServo = opMode.hardwareMap.servo.get("servoMarker");

        //configuring motors to their appropriate direction
        motors = new DcMotor[] {leftMotor, rightMotor, strafeMotor, liftMotor};
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //resetting encoder counts and run modes of motors
        for(DcMotor motor: motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        detector = new GoldMineralDetector();
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

        detector.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
    }

    //method used for displaying telemetry information during the tele-operated modes
    public void telemetry() {
        opMode.telemetry.addData("Motors", "left: (%.2f), right: (%.2f), strafe: (%.2f), lift: (%.2f)", leftMotor.getPower(), rightMotor.getPower(), strafeMotor.getPower(), liftMotor.getPower());
        opMode.telemetry.addData("Heading", getHeading());
        opMode.telemetry.update();
    }

    //method used for displaying telemetry information during the autonomous period
    public void autonomousTelemetry() {
        opMode.telemetry.addData("Motors", "left: (%.2f), right: (%.2f), strafe: (%.2f), lift: (%.2f)", leftMotor.getPower(), rightMotor.getPower(), strafeMotor.getPower(), liftMotor.getPower());
        opMode.telemetry.addData("Encoders", "left: (%d), right: (%d), strafe: (%d), lift: (%d)", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition(), strafeMotor.getCurrentPosition(), liftMotor.getCurrentPosition());
        opMode.telemetry.addData("Targets", "left: (%d), right: (%d), strafe: (%d), lift: (%d)", leftMotor.getTargetPosition(), rightMotor.getTargetPosition(), strafeMotor.getTargetPosition(), liftMotor.getTargetPosition());
        opMode.telemetry.addData("Gold Position", detector.getScreenPosition());
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
    public void setStrafePower(double power) {
        strafeMotor.setPower(power);
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
    public void setStrafeTargetPosition(double target) {
        withEncoder();
        strafeMotor.setTargetPosition(Calculator.getTicks(target));
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

    public void delatch() {
        withEncoder();
        liftMotor.setTargetPosition(Constants.AUTO_MAX_LIFT_TICKS);
        liftMotor.setPower(Constants.LIFT_SPEED);
        while(liftMotor.isBusy());
        liftMotor.setPower(0);
        withoutEncoder();
    }

    public void latch() {
        withEncoder();
        liftMotor.setTargetPosition(-Constants.AUTO_MAX_LIFT_TICKS);
        liftMotor.setPower(Constants.LIFT_SPEED);
        while(liftMotor.isBusy());
        liftMotor.setPower(0);
        withoutEncoder();

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
        setPower(Constants.DRIVE_SPEED);
        pidController.reset();
        pidController.pidCoefficients = Constants.PID_DRIVE_COEFFICIENTS;
        pidController.targetPosition = angle;

        while(leftMotor.isBusy() && rightMotor.isBusy()) {
            double currentAngle = getHeading();
            double steer = pidController.update(currentAngle);

            if (inches < 0)
                steer *= -1.0;

            double leftSpeed = Constants.DRIVE_SPEED + steer;
            double rightSpeed = Constants.DRIVE_SPEED - steer;

            setPower(leftSpeed, rightSpeed);
        }
        setPower(0);
        withoutEncoder();
    }

    public void rotate(double degrees) {
        double targetAngle = degrees + getHeading();

        if(targetAngle > 180)
            targetAngle -= 360;
        if(targetAngle < 180)
            targetAngle += 180;

        pidController.reset();
        pidController.pidCoefficients = Constants.PID_TURN_COEFFICIENTS;
        pidController.targetPosition = targetAngle;

        double heading;
        while((heading = getHeading()) != targetAngle) {
            double steer = pidController.update(heading);
            setPower(Constants.TURN_SPEED + steer, Constants.TURN_SPEED - steer);
        }
        setPower(0);
    }

    public void strafe(double inches) {
        withEncoder();
        int moveTicks = Calculator.getTicks(inches);
        setStrafeTargetPosition(inches);
        setStrafePower(Constants.DRIVE_SPEED);
        pidController.reset();
        pidController.pidCoefficients = Constants.PID_STRAFE_COEFFICIENTS;
        pidController.targetPosition = getHeading();

        while(strafeMotor.isBusy()) {
            double steer = pidController.update(getHeading());
            setPower(Constants.TURN_SPEED + steer, Constants.TURN_SPEED - steer);
        }
        setPower(0, 0, 0);
        withEncoder();
    }
}
