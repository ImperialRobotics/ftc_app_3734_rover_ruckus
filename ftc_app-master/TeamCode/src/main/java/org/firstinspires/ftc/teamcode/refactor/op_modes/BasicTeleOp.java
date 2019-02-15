package org.firstinspires.ftc.teamcode.refactor.op_modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.refactor.robot.Robot;
import org.firstinspires.ftc.teamcode.refactor.utils.Constants;

/*
 * Controls:
 * left_stick_y - drive
 * right_stick_x - turn
 * left_stick_x - strafe
 *
 * dpad_up - lift up
 * dpad_down - lift down
 *
 * right_bumper - rotate to max
 * left_bumper - rotate to mins
 * dpad_right - extend
 * dpad_left - retract
 * right_trigger - intake mineral
 * left_trigger - spit out mineral
 *
 * y - go to "neutral" intake position
 * x - go to "intake" intake position
 * b - go to "deposit" intake position
 * */

@TeleOp(name = "Basic Test TeleOp", group = "Linear OpMode")
public class BasicTeleOp extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);

        while(opModeIsActive()) {
            double left  = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * 0.5;
            double right = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * 0.5;

            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }
            robot.setPower(left, right);

            //controlling lift
            if(gamepad1.dpad_up)
                robot.liftMotor.setPower(Constants.LIFT_SPEED);
            else if(gamepad1.dpad_down)
                robot.liftMotor.setPower(-Constants.LIFT_SPEED);
            else if(!gamepad1.dpad_up && !gamepad1.dpad_down)
                robot.liftMotor.setPower(0);

            //controlling rotation of entire arm
            if(gamepad1.right_bumper)
                robot.intake.rotateMotor.setPower(Constants.ROTATE_SPEED);
            else if(gamepad1.left_bumper)
                robot.intake.rotateMotor.setPower(-Constants.ROTATE_SPEED);
            else if(!gamepad1.right_bumper && !gamepad1.left_bumper)
                robot.intake.rotateMotor.setPower(0);

            //controlling extension of arm
            if(gamepad1.dpad_right)
                robot.intake.extendMotor.setPower(Constants.EXTEND_SPEED);
            else if(gamepad1.dpad_left)
                robot.intake.extendMotor.setPower(-Constants.EXTEND_SPEED);
            else if(!gamepad1.dpad_right && !gamepad1.dpad_left)
                robot.intake.extendMotor.setPower(0);

            //controlling intake (surgical tubing axle) of arm
            if(gamepad1.right_trigger > 0)
                robot.intake.intakeMotor.setPower(Constants.INTAKE_SPEED);
            if(gamepad1.left_trigger > 0)
                robot.intake.intakeMotor.setPower(-Constants.INTAKE_SPEED);
            else if(!(gamepad1.right_trigger > 0) && !(gamepad1.left_trigger > 0))
                robot.intake.intakeMotor.setPower(0);
        }
    }
}
