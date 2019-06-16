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
@Autonomous(name = "AutoAltNoPark", group = "MaximumOverdrive")
public class AutoAltNoPark extends UnitBotMain
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

        initialize();

        while (!isStarted()) {
            telemetry.addLine("Ready, Waiting for Start");
            telemetry.update();
            //_motorStronkBoi.setPower(.7);
            sleep(5);
        }

        double initialHeading = getCurrentHeading();
        int mvalue = 0;
        int tvalue = 0;
        double cubexPos = 0;
        double cubeyPos = 0;
        int count = 0;

        //Land

        _motorExpand1.setPower(-1);
        _motorExpand2.setPower(1);
        sleep(550);
        _motorExpand1.setPower(-.03);
        _motorExpand2.setPower(.03);
        pivot(-2000);
        _servoHook.setPosition(0);
        _servoGrab1.setPosition(.4);
        sleep(750);
        initialHeading = getCurrentHeading();
        detector.enable();
        pivot(600);
        _servoHook.setPosition(1);
        drive(new SimpleDriver(.3, 50, null), false, true);

        //Detect cube

        cubexPos = detector.getXPosition();
        cubeyPos = detector.getYPosition();
        int side1 = 0;
        int side2 = 0;
        int side3 = 0;
        while(count < 150){
            cubexPos = detector.getXPosition();
            count ++;
            telemetry.addData("Cube xPos: ", cubexPos);
            telemetry.addData("Cube yPos: ", cubeyPos);
            telemetry.update();
            if(cubexPos < 180){
                side1++;
            }else if(cubexPos >= 180 && cubexPos <= 260){
                side2++;
            }else if(cubexPos > 260){
                side3++;
            }
            sleep(5);
        }
        detector.disable();
        telemetry.addData("side1: ", side1);
        telemetry.addData("side2: ", side2);
        telemetry.addData("side3: ", side3);
        telemetry.update();
        int side = 0;
        if(side1 > side2){
            if(side1 > side3){
                side = 1;
            }else{
                side = 3;
            }
        }else{
            if(side2 > side3){
                side = 2;
            }else{
                side = 3;
            }
        }

        //Go to depot for claim
        drive(new SimpleDriver(.7, 800, null), false, true);

        //claim
        pivot(500);
        drive(new SimpleDriver(.3, 100, null), false, true);
        extend(300);
        _motorIntake.setPower(1);
        sleep(175);
        _motorIntake.setPower(0);

        //drive to lander

        drive(new SimpleDriver(-.7, 1000, null), false, true);
        _motorIntake.setPower(-.5);

        //Pick up cube

        RobotLog.d("DepotAuton::drive");
        if(side == 1) {
            turn(new IMUTurner(23, .4, _imu1, 1, null), true, true);
        }else if(side == 3) {
            turn(new IMUTurner(-29, .4, _imu1, 1, null), true, true);
        }else if(side == 2) {
            turn(new IMUTurner(-4, .4, _imu1, 1, null), true, true);
        }
        pivot(1800);
        _motorIntake.setPower(-.8);
        if(!(side==2)) {
            extend(850);
            sleep(250);
            extend(-100);
        }else{
            extend(750);
            sleep(250);
        }

        //Deposit first cube

        pivot(-600);
        extend(-1000);
        _servoHook.setPosition(1);
        turn(new IMUTurner(initialHeading - getCurrentHeading(), .6, _imu1, 1, null), true, true);
        pivot(-3150);
        extend(1000);
        _servoGrab1.setPosition(1);
        sleep(1000);
        _servoGrab1.setPosition(.4);
        turn(new IMUTurner(initialHeading - getCurrentHeading(), .6, _imu1, 1, null), true, true);

//        extend(-800);
//
//        pivot(3100);
//
//        drive(new SimpleDriver(-.7, 900, null), false, true);
//
//        turn(new IMUTurner(initialHeading - getCurrentHeading() - 90, .6, _imu1, 1, null), true, true);
//
//        drive(new SimpleDriver(-1, 4000, null), false, true);




    }

}
