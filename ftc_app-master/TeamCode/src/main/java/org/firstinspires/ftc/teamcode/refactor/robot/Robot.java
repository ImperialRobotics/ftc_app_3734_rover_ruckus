package org.firstinspires.ftc.teamcode.refactor.robot;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.refactor.utils.Calculator;
import org.firstinspires.ftc.teamcode.refactor.utils.Constants;

public class Robot {
    public DcMotor leftMotor, rightMotor, strafeMotor, liftMotor;
    public DcMotor[] motors;
    public Servo markerServo;

    public Intake intake;
    public GoldMineralDetector detector;
    public Audio audio;

    public BNO055IMU imu;
    public LinearOpMode opMode;

    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;

    //----------------------------------------------------------------------------------------------
    // Initialization Methods
    //----------------------------------------------------------------------------------------------

    public Robot(LinearOpMode opMode) {
        leftMotor = opMode.hardwareMap.dcMotor.get("motorLeft");
        rightMotor = opMode.hardwareMap.dcMotor.get("motorRight");
        strafeMotor = opMode.hardwareMap.dcMotor.get("motorMiddle");
        liftMotor = opMode.hardwareMap.dcMotor.get("motorArm");
        markerServo = opMode.hardwareMap.servo.get("servoMarker");

        motors = new DcMotor[] {leftMotor, rightMotor, strafeMotor, liftMotor};
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        for(DcMotor motor: motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake = new Intake(opMode);
        detector = new GoldMineralDetector();
        audio = new Audio(opMode);

        this.opMode = opMode;
    }

    public void initAutonomous() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        opMode.telemetry.addData("Status", "calibrating imu...");
        opMode.telemetry.update();

        while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
            opMode.sleep(50);
            opMode.idle();
        }

        detector.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
    }

    public void disableAutonomous() {
        detector.disable();
        resetAngle();
        for(DcMotor motor: motors)
            motor.setPower(0.0);
        intake.setPower(0.0, 0.0, 0.0);
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    public void telemetry() {
        //drivetrain and lift motor powers
        opMode.telemetry.addData("Motors", "left: (%.2f), right: (%.2f), strafe: (%.2f), lift: (%.2f)",
                leftMotor.getPower(), rightMotor.getPower(), strafeMotor.getPower(), liftMotor.getPower());

        //intake motor powers
        opMode.telemetry.addData("Intake Motors", "rotate: (%.2f), extend: (%.2f), intake: (%.2f)",
                intake.rotateMotor.getPower(), intake.extendMotor.getPower(), intake.intakeMotor.getPower());

        opMode.telemetry.addData("Heading", getAngle());
        opMode.telemetry.update();
    }

    public void autonomousTelemetry() {
        //motor powers
        opMode.telemetry.addData("Motors", "left: (%.2f), right: (%.2f), strafe: (%.2f), lift: (%.2f), rotate: (%.2f), extend: (%.2f), intake: (%.2f)",
                leftMotor.getPower(), rightMotor.getPower(), strafeMotor.getPower(), liftMotor.getPower(),
                intake.rotateMotor.getPower(), intake.extendMotor.getPower(), intake.intakeMotor.getPower());

        //motor encoder positions
        opMode.telemetry.addData("Encoders", "left: (%d), right: (%d), strafe: (%d), lift: (%d), rotate: (%d), extend: (%d), intake: (%d)",
                leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition(), strafeMotor.getCurrentPosition(), liftMotor.getCurrentPosition(),
                intake.rotateMotor.getCurrentPosition(), intake.extendMotor.getCurrentPosition(), intake.intakeMotor.getCurrentPosition());

        //motor encoder target positions
        opMode.telemetry.addData("Targets", "left: (%d), right: (%d), strafe: (%d), lift: (%d), rotate: (%d), extend: (%d), intake: (%d)",
                leftMotor.getTargetPosition(), rightMotor.getTargetPosition(), strafeMotor.getTargetPosition(), liftMotor.getTargetPosition(), intake.rotateMotor.getTargetPosition(), intake.extendMotor.getTargetPosition(), intake.intakeMotor.getTargetPosition());

        //screen x - position of gold mineral
        opMode.telemetry.addData("Gold Position", detector.getScreenPosition());
        opMode.telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Helper Methods
    //----------------------------------------------------------------------------------------------

    private void withEncoder() {
        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    private void withoutEncoder() {
        for(DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

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

    //----------------------------------------------------------------------------------------------
    // Autonomous Methods
    //----------------------------------------------------------------------------------------------

    public void delatch() {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(-Constants.AUTO_MAX_LIFT_TICKS);
        liftMotor.setPower(Constants.LIFT_SPEED);
        while(liftMotor.isBusy());
        liftMotor.setPower(0);
        withoutEncoder();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    private double checkDirection() {
        double angle = getAngle();
        double correction = ((angle == 0) ? 0 : -angle) * .10;
        return correction;
    }

    public void rotate(int degrees) {
        resetAngle();
        double leftPower = (degrees < 0) ? -power : power;
        double rightPower = (degrees < 0) ? power : -power;
        if(degrees == 0) return;

        setPower(leftPower, rightPower);

        if (degrees < 0) {
            while (opMode.opModeIsActive() && getAngle() == 0);
            while (opMode.opModeIsActive() && getAngle() > degrees);
        }
        else
            while (opMode.opModeIsActive() && getAngle() < degrees);

        setPower(0);
        opMode.sleep(500);
        resetAngle();
    }

    public void drive(double inches) {
        int ticks = Calculator.getTicks(inches);
        setTargetPosition(ticks, ticks);
        setPower(Constants.DRIVE_SPEED, Constants.DRIVE_SPEED);
        while(leftMotor.isBusy() && rightMotor.isBusy());
        setPower(0);
        withoutEncoder();
    }
}