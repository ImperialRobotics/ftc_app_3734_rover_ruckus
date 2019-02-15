package org.firstinspires.ftc.teamcode.refactor.op_modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.refactor.robot.Robot;
import org.firstinspires.ftc.teamcode.refactor.utils.AutonomousRecorder;

//Controls:
//y - record center
//b - record right
//x - record left
//dpad_left - play left
//dpad_right - play right
//dpad_up - play center
//right_bumper - stop recording

@TeleOp(name = "Recorded Autonomous", group = "Linear OpMode")
public class RecordedAutonomous extends LinearOpMode {
    Robot robot;
    AutonomousRecorder recorder;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        recorder = new AutonomousRecorder(robot, this);

        while(opModeIsActive()) {
            if(gamepad1.x)
                recorder.record("Left");
            if(gamepad1.y)
                recorder.record("Center");
            if(gamepad1.b)
                recorder.record("Right");

            if(gamepad1.dpad_left)
                recorder.play("Left");
            if(gamepad1.dpad_up)
                recorder.play("Center");
            if(gamepad1.dpad_right)
                recorder.play("Right");
        }
    }
}
