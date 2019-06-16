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
@Autonomous(name = "BigBoiAuton", group = "MaximumOverdrive")
public class BigBoiAuton extends UnitBotMain
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

        _frontRight = hardwareMap.dcMotor.get("front_right");
        _backRight = hardwareMap.dcMotor.get("back_right");
        _frontLeft = hardwareMap.dcMotor.get("front_left");
        _backLeft = hardwareMap.dcMotor.get("back_left");

        initialize();

        while (!isStarted()) {
            telemetry.addLine("Ready, Waiting for Start");
            telemetry.update();
            //_motorStronkBoi.setPower(.7);
            sleep(5);
        }

        _backRight.setPower(0.7);
        _frontRight.setPower(0.7);
        _backLeft.setPower(-0.7);
        _frontLeft.setPower(-0.7);
        sleep(1750);

        _backRight.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _frontLeft.setPower(0);
        sleep(100);

        turn(new IMUTurner(-10, .3, _imu1, 2, null), true, true);


    }
    }
