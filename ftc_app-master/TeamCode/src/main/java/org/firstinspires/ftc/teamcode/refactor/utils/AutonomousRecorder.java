package org.firstinspires.ftc.teamcode.refactor.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.refactor.robot.Robot;

import java.util.*;
import java.io.*;

//Control
//dpad_up -> drive forward 10 inches
//dpad_down -> drive backward 10 inches
//dpad_right -> rotate 90 degrees
//dpad_left -> rotate -90 degrees
//right_stick_button -> rotate 10 degrees
//left_stick_button -> rotate -10 degrees
//right_bumper -> stop recording


public class AutonomousRecorder {
    private PrintWriter out;
    private Scanner in;
    private Robot robot;
    private Gamepad gamepad1;
    private Telemetry telemetry;

    public AutonomousRecorder(Robot robot, LinearOpMode opMode) {
        this.robot = robot;
        gamepad1 = opMode.gamepad1;
        telemetry = opMode.telemetry;
    }

    public void record(String filepath) {
        telemetry.addData("Status", "recording \"" + filepath + "\"");
        telemetry.update();

        try {
            out = new PrintWriter(new FileWriter("/sdcard/FIRST/" + filepath + ".txt"));
        } catch (IOException exception) {
            telemetry.addData("Status", "IOException thrown");
            telemetry.update();
        }

        List<String> strings = new ArrayList<String>();

        while(!gamepad1.right_bumper) {
            if(gamepad1.dpad_up) {
                robot.drive(10);
                strings.add("drive 10");
            }
            if(gamepad1.dpad_down) {
                robot.drive(-10);
                strings.add("drive -10");
            }
            if(gamepad1.dpad_right) {
                robot.rotate(90);
                strings.add("rotate 90");
            }
            if(gamepad1.dpad_left) {
                robot.rotate(-90);
                strings.add("rotate -90");
            }
            if(gamepad1.right_stick_button) {
                robot.rotate(10);
                strings.add("rotate 10");
            }
            if(gamepad1.left_stick_button) {
                robot.rotate(-10);
                strings.add("rotate -10");
            }
        }

        for(String str : strings)
            out.println(str);

        telemetry.addData("Status", "finished recording \"" + filepath + "\"");
        telemetry.update();

        telemetry.addData("Status", "finish writing \"" + filepath + "\"");
        telemetry.update();
    }

    public void play(String filepath) {
        telemetry.addData("Status", "playing \"" + filepath + "\"");
        telemetry.update();

        try {
            in = new Scanner(new BufferedReader(new FileReader("/sdcard/FIRST/" + filepath + ".txt")));

            robot.delatch();

            while(in.hasNextLine()) {
                switch(in.nextLine()) {
                    case "drive 10":
                        robot.drive(10);
                        break;
                    case "drive -10":
                        robot.drive(-10);
                        break;
                    case "rotate 90":
                        robot.rotate(90);
                        break;
                    case "rotate -90":
                        robot.rotate(-90);
                        break;
                    case "rotate 10":
                        robot.rotate(10);
                        break;
                    case "rotate -10":
                        robot.rotate(-10);
                        break;
                }
            }
        } catch (IOException e) {
            telemetry.addData("Status", "IOException thrown");
            telemetry.update();
        }

        telemetry.addData("Status", "finished recording \"" + filepath + "\"");
        telemetry.update();
    }
}
