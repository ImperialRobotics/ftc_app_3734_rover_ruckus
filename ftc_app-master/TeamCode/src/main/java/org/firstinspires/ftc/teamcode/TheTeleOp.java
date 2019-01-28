package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;


@TeleOp(name = "TheTeleOp", group = "TeleOp")


public class TheTeleOp extends OpMode
{

    HardwareRobot hr = new HardwareRobot();

    public void init()
    {
        hr.init(hardwareMap);
        hr.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hr.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //hr.motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Say", "Initialize Complete");
        updateTelemetry(telemetry);
    }

    public void init_loop()
    {



    }


    public void start()
    {

    }


    public void loop()
    {
        double left  = -gamepad1.left_stick_y - gamepad1.right_stick_x;
        double right = -gamepad1.left_stick_y + gamepad1.right_stick_x;

        left = left * 0.5;
        right = right * 0.5;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        hr.motorLeft.setPower(left);
        hr.motorRight.setPower(right);


        hr.motorMiddle.setPower(gamepad1.left_stick_x);
        hr.motorArm.setPower(0);
        if(gamepad1.left_bumper){
            hr.motorArm.setPower(-1);        }
        else{
            if(gamepad1.right_bumper){
                hr.motorArm.setPower(1);
            }
        }

        if(gamepad1.b){
            hr.servoMarker.setPosition(0.4);
        }
        if(gamepad1.a){
            hr.servoMarker.setPosition(0);
        }
        if(gamepad1.x){
            hr.servoMarker.setPosition(1);
        }


        int position = hr.motorArm.getCurrentPosition();
        telemetry.addData("Encoder Position", position);
    }


    public void stop()
    {

    }
}