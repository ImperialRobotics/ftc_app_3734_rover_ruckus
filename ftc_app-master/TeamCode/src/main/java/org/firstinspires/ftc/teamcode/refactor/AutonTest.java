package org.firstinspires.ftc.teamcode.refactor;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Autonomous Testing", group = "Linear opMode")
public class AutonTest extends LinearOpMode {

    Robot robot = new Robot(this);
    @Override
    public void runOpMode() {
        robot.initAutonomous();

        if(gamepad1.dpad_up)
            robot.drive(30);
        if(gamepad1.dpad_down)
            robot.drive(-30);

        if(gamepad1.dpad_right)
            robot.rotate(90);
        if(gamepad1.dpad_left)
            robot.rotate(-90);

        robot.autonomousTelemetry();
    }
}
