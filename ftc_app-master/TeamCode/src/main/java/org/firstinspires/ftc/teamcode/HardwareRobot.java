package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareRobot {

    //drive train
    public DcMotor motorRight = null;
    public DcMotor motorLeft = null;
    public DcMotor motorMiddle = null;
    public  DcMotor motorArm = null;
    public Servo servoMarker = null;


    private ElapsedTime period  = new ElapsedTime();

    HardwareMap hwMap = null;

    public HardwareRobot()
    {

    }

    public void init(HardwareMap ahwMap)
    {
        //initialize hardware

        hwMap = ahwMap;

        motorRight = hwMap.dcMotor.get("motorRight");
        motorLeft = hwMap.dcMotor.get("motorLeft");
        motorMiddle = hwMap.dcMotor.get("motorMiddle");
        motorArm = hwMap.dcMotor.get("motorArm");

        servoMarker = hwMap.servo.get("servoMarker");


        motorRight.setPower(0);
        motorLeft.setPower(0);

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void waitForTick(long periodMs)  throws InterruptedException
    {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular shoot period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the shoot clock for the next pass.
        period.reset();
    }

}
