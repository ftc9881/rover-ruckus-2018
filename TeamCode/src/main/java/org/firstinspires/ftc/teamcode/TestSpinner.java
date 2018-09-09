//package org.firstinspires.ftc.teamcode;
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.RobotLog;
//
///*
// * An example linear op mode where the pushbot
// * will drive in a square pattern using sleep()
// * and a for loop.
// */
//@Autonomous(name = "TestSpinner", group = "test")
//public class TestSpinner extends OctobotMain
//{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addLine("Initializing Motors and Sensors");
//        telemetry.update();
//
//        initialize();
//
//        telemetry.addLine("Ready");
//        telemetry.update();
//
//        waitForStart();
//
//        Servo grabberServo = getGrabberServo();
//
//        MotorRunner spinner = new MotorRunner(_motorSpinner, -.2, 0, new SpinnerStopper(_sensorRGB, .5f, new TimeStopper(2500, null), grabberServo == _servoGrabberBlue, 1));
//        spinner.startMotor();
//
//        while(spinner.isRunning() && opModeIsActive()) {
//            sleep(5);
//        }
//
//    }
//
//}
