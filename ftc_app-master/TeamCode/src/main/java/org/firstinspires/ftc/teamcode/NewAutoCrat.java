package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "NewAutonomousCrater" , group = "Autonomous")
public class NewAutonomousCrat extends LinearOpMode {

    HardwareRobot robot   = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    //motor/drive train
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    //dogeCV detector
    private GoldAlignDetector detector;

    //IMU variables
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    @Override
    //init is hit
    public void runOpMode(){
        //init hardware
        robot.init(hardwareMap);

        //imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        //init imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        //reset encoders of motors
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.motorRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.motorLeft.getCurrentPosition(),
                robot.motorRight.getCurrentPosition());
        telemetry.update();

        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in
        // which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        hang();
        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        down();
        sleep(3000);
        rotate(-9,0.3);
        sleep(750);
        //encoderDrive(1, -1,1,2);
        encoderDrive(1,-1,-1,2);
        rotate(6,0.3);
        //encoderDrive(1, 1,-1.,2);
        sleep(500);

        detector.enable();
        sleep(3000);

        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.

        //middle
        if (detector.isFound() && detector.getXPosition() > 200){
            encoderDrive(0.75,-10,-10,5);
            sleep(500);
            encoderDrive(1,6 ,6,6);
            rotate(70,0.3);
            encoderDrive(0.75,-16.5 ,-16.5,6);
            rotate(40,0.3);
            encoderDrive(0.75,-12,-12,9);
            score(-1330,0.75);
            sleep(1000);
            encoderDrive(0.75,17,17,9);
            rotate(8,0.3);
            encoderDrive(0.25,2,2,3);
            //encoderDrive(0,0,0,30);
            //sleep(2000);6
            //rotate(50,power);
            //sleep(1000);
            //encoderDrive(1,-6,-6, 5);
            //sleep(1000);
            //rotate(-10,power);
            //sleep(1000);
            //encoderDrive(1,-12,-12,10);
            //sleep(3000);
            //rotate(25,power);
            //sleep(1000);
            //encoderDrive(1,-4,-4, 30);
            //encoderDrive(1,1.9,-1.9,5);
            //encoderDrive(1, -16,-16,5);

        }
        //left
        else if (detector.isFound() && detector.getXPosition() < 200){
            rotate(15,0.5);
            encoderDrive(0.75,-10,-10,10);
            sleep(500);
            encoderDrive(0.75,4 ,4,6);
            rotate(51,0.3);
            encoderDrive(0.75,-16,-16,10);
            rotate(32,0.3);
            //changes to 1
            encoderDrive(1,-12,-12,12);
            score(-1330,0.75);
            sleep(500);
            encoderDrive(1,20,20,10);

            encoderDrive(0.25,5,5,10);
            /*
            encoderDrive(0.75,-16,-16,5);
            rotate(-65,.3);
            encoderDrive(0.75,-9,-9,9);
            sleep(1000);
            score(-1300,0.75);
            sleep(1000);
            encoderDrive(0.75,24,24,10);
            sleep(1000);
            */
            //rotate(-25,0.3);
            //encoderDrive(0.75,9,9,10);

           /*
            rotate(15,0.5);
            //encoderDrive(1,1,-1,5);
            encoderDrive(1,12,12,5);
            //encoderDrive(1,-2.6,2.6,5);
            rotate(-60,power);
            encoderDrive(1,7,7  ,10);
            sleep(1000);
            encoderDrive(1,-10,-10,10);
            rotate(-3,0.4);
            encoderDrive(1,-7,-7,5);
            rotate(-7,0.4);
            encoderDrive(1,-4,-4,5);
            */
        }
        //right
        else if(!detector.isFound()){
            rotate(-22,0.3);
            encoderDrive(0.75,-10,-10,5);
            sleep(500);
            encoderDrive(1,6,6,5);
            rotate(109,0.3);
            encoderDrive(0.75,-19.5,-19.5,5);
            rotate(30,0.3);
            encoderDrive(1,-12,-12,10);
            score(-1330,0.3);
            sleep(1000);
            encoderDrive(1,19,19,10);
            rotate(14,1);
            encoderDrive(1,4,4,10);

            
            //score(-1300,0.75);
            //sleep(1000);
            //encoderDrive(0.75,20,20,5);
            /*
            //encoderD4 rive(1,-1.6,1.6,5);
            rotate(-22,0.5);
            encoderDrive(1,13,13,5);
            sleep(1000);
            rotate(55,power);
            sleep(1000);
            encoderDrive(1,5,5,20);
            sleep(1000);
            rotate(-5,power);
            sleep(1000);
            encoderDrive(1,-12,-12,40);
            sleep(1000);
            rotate(55,power);
            sleep(1000);
            encoderDrive(1,-9.,-9,5);
            sleep(1000);
            encoderDrive(0,0,0,50);
            */
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {
                correction = checkDirection();
                correction = checkDirection();
                robot.motorLeft.setPower(Math.abs(speed+correction));
                robot.motorRight.setPower(Math.abs(speed+correction));
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void hang(){
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorArm.setTargetPosition(0);
        robot.motorArm.setPower(-1);
        int position = robot.motorArm.getCurrentPosition();
        telemetry.addData("Encoder Position", position);
        telemetry.update();
    }

    public void down(){
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorArm.setTargetPosition(-7700);//2432
        robot.motorArm.setPower(-1);
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.motorLeft.setPower(leftPower);
        robot.motorRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }


    public void score(int encoder, double speed){
        robot.motorRotate.setTargetPosition(encoder);//2432
        robot.motorRotate.setPower(speed);
    }

}
