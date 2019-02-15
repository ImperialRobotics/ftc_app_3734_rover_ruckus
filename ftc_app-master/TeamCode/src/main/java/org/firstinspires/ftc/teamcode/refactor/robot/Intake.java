package org.firstinspires.ftc.teamcode.refactor.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.refactor.utils.Constants;

public class Intake {
    private OpMode opMode;
    public DcMotor rotateMotor, extendMotor, intakeMotor;
    private DcMotor[] motors;

    public Intake(LinearOpMode opMode) {
        rotateMotor = opMode.hardwareMap.dcMotor.get("rotateMotor");
        extendMotor = opMode.hardwareMap.dcMotor.get("extendMotor");
        intakeMotor = opMode.hardwareMap.dcMotor.get("intakeMotor");

        motors = new DcMotor[] {rotateMotor, extendMotor, intakeMotor};
        for(DcMotor motor: motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        this.opMode = opMode;
    }

    //----------------------------------------------------------------------------------------------
    // Helper Methods
    //----------------------------------------------------------------------------------------------

    public void setPower(double power) {
        for(DcMotor motor: motors)
            motor.setPower(power);
    }
    public void setPower(double rotatePower, double extendPower, double intakePower) {
        rotateMotor.setPower(rotatePower);
        extendMotor.setPower(extendPower);
        intakeMotor.setPower(intakePower);
    }

    private static void runToPosition(DcMotor motor, int targetPosition, double power, boolean reset) {
        if(reset)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(targetPosition);
        motor.setPower(power);
        while(motor.isBusy());
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //----------------------------------------------------------------------------------------------
    // Run-To-Position Methods
    //----------------------------------------------------------------------------------------------

    public void extend() { runToPosition(extendMotor, Constants.MAX_EXTEND_TICKS, Constants.EXTEND_SPEED, false); }
    public void retract() { runToPosition(extendMotor, 0, Constants.EXTEND_SPEED, false); }

    public void intakeMineral() { runToPosition(intakeMotor, (int) Constants.INTAKE_TICKS_PER_REV, Constants.INTAKE_SPEED, true); }
    public void spitMineral() { runToPosition(intakeMotor,(int) -Constants.INTAKE_TICKS_PER_REV, Constants.INTAKE_SPEED, true); }

    public void rotateMax() { runToPosition(rotateMotor, (int) Constants.MAX_ROTATE_TICKS, Constants.ROTATE_SPEED, false); }
    public void rotateMin() { runToPosition(rotateMotor, 0, Constants.ROTATE_SPEED, false); }

    public void goToPosition(Position position) {
        retract();
        switch(position) {
            case NEUTRAL:
                rotateMin();
                break;
            case INTAKE:
                rotateMax();
                extend();
                break;
            case DEPOSIT:
                rotateMin();
                extend();
                break;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous Methods
    //----------------------------------------------------------------------------------------------

    public void intake() {
        rotateMax();
        extend();
        intakeMineral();
    }

    public void deposit() {
        retract();
        rotateMin();
        extend();
        spitMineral();
    }

    //----------------------------------------------------------------------------------------------
    // Getters and Setters
    //----------------------------------------------------------------------------------------------

    public OpMode getOpMode() {
        return opMode;
    }
    public void setOpMode(OpMode opMode) {
        this.opMode = opMode;
    }
    public DcMotor[] getMotors() {
        return motors;
    }
    public void setMotors(DcMotor[] motors) {
        this.motors = motors;
    }

    //----------------------------------------------------------------------------------------------
    // Position (Value Class for Set Intake Positions)
    //----------------------------------------------------------------------------------------------

    public static enum Position {
        NEUTRAL, INTAKE, DEPOSIT;
    }
}