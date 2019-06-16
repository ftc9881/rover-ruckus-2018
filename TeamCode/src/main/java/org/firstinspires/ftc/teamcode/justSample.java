package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep()
 * and a for loop.
 */
@Disabled
@Autonomous(name = "justSample", group = "MaximumOverdrive")
public class justSample extends UnitBotMain
{
    private GoldAlignDetector detector = new GoldAlignDetector();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
        telemetry.addData("X Pos" , detector.getXPosition());

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 20; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;



        double cubexPos = 0;
        double cubeyPos = 0;
        int count = 0;

        initialize();

        while (!isStarted()) {
            telemetry.addLine("Ready, Waiting for Start");
            telemetry.update();
            sleep(5);
        }

        //waitForStart();


        double initialHeading = getCurrentHeading();

        int mvalue = 0;

        int tvalue = 0;

        mvalue = _motorStronkBoi.getCurrentPosition();

        tvalue = mvalue - 2850;

        _motorStronkBoi.setPower(-.7);

        while(_motorStronkBoi.getCurrentPosition() > tvalue && count < 150){
            sleep(5);
            count++;
            telemetry.addData("Encoder: ", _motorStronkBoi.getCurrentPosition());
            telemetry.addData("Count: ", count);
            telemetry.update();
        }

        _motorStronkBoi.setPower(0);

        _servoHook.setPosition(0);

        sleep(1500);

        mvalue = _motorStronkBoi.getCurrentPosition();

        tvalue = mvalue + 700;

        _motorStronkBoi.setPower(.7);

        while(_motorStronkBoi.getCurrentPosition() < tvalue && count < 150){
            sleep(5);
            count++;
            telemetry.addData("Encoder: ", _motorStronkBoi.getCurrentPosition());
            telemetry.addData("Count: ", count);
            telemetry.update();
        }

        tvalue = _motorStronkBoi.getCurrentPosition();

        _servoHook.setPosition(1);

        _motorStronkBoi.setPower(1);

        _motorStronkBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        count = 0;
        while(count < 50) {
            _motorStronkBoi.setTargetPosition(tvalue);
            count ++;
        }

        _motorExpand1.setPower(-1);
        _motorExpand2.setPower(1);

        sleep(550);

        _motorExpand1.setPower(0);
        _motorExpand2.setPower(0);

        detector.enable();

        cubexPos = detector.getXPosition();
        cubeyPos = detector.getYPosition();

        while(count < 2000){
            cubexPos = detector.getXPosition();
            count ++;
            telemetry.addData("Cube xPos: ", cubexPos);
            telemetry.addData("Cube yPos: ", cubeyPos);
            telemetry.update();
            sleep(5);
        }

        detector.disable();

        telemetry.addData("Cube xPos: ", cubexPos);
        telemetry.addData("Cube yPos: ", cubeyPos);
        telemetry.update();

        count = 0;


        int side = 0;



        if(cubexPos < 180){
            side = 1;
        }else if(cubexPos >= 180 && cubexPos <= 260){
            side = 2;
        }else if(cubexPos > 260){
            side = 3;
        }



        sleep(1500);

        if(side == 1) {
            turn(new IMUTurner(25, .1, _imu1, 2, null), true, true);
        }else if(side == 3) {
            turn(new IMUTurner(-25, .1, _imu1, 2, null), true, true);
        }

        telemetry.addData("Side: ", side);
        telemetry.update();

        sleep(500);

        _motorStronkBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mvalue = _motorStronkBoi.getCurrentPosition();

        tvalue = mvalue + 2200;

        _motorStronkBoi.setPower(.7);

        while(_motorStronkBoi.getCurrentPosition() < tvalue && count < 150){
            sleep(5);
            count++;
            telemetry.addData("Encoder: ", _motorStronkBoi.getCurrentPosition());
            telemetry.addData("Count: ", count);
            telemetry.update();
        }

        _motorStronkBoi.setPower(0);



        _motorIntake.setPower(-1);

//        initialHeading = getCurrentHeading();

        //drive(new IMUDriver(.3, 0, _imu1, 0, initialHeading, 1500, null), true, true);

        _motorIntake.setPower(-1);

        _motorExpand1.setPower(-1);
        _motorExpand2.setPower(1);

        sleep(1550);

        _motorExpand1.setPower(0);
        _motorExpand2.setPower(0);

        sleep(2000);

        _motorIntake.setPower(0);



    }

}
