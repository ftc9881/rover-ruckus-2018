//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//
//@Autonomous(name = "OctobotAutonomousBlueSide", group = "octobot")
//public class OctobotAutonomousBlueSide extends OctobotMain
//{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addLine("Initializing Motors and Sensors");
//        telemetry.update();
//
//        initialize();
//
//        telemetry.addLine("Initializing VuForia");
//        telemetry.update();
//
//        initializeVuforia();
//        startVuforia();
//
//        double initialHeading = getCurrentHeading();
//
//        float[] hsvValues = new float[3];
//        final float values[] = hsvValues;
//
//        Servo grabberServo = getGrabberServo();
//
//        Servo otherGrabber;
//
//        if (grabberServo == _servoGrabberBlue) {
//            otherGrabber = _servoGrabberRed;
//        } else if (grabberServo == _servoGrabberRed) {
//            otherGrabber = _servoGrabberBlue;
//        } else {
//            otherGrabber = null;
//        }
//
//        RobotLog.d("OctobotAutonomousBlueSide::initialHeading::" + initialHeading);
//
////        while (!_button3.getState()){
////            RobotLog.d("OctobotAutonomousBlueSide::_button3.getState()::" + _button3.getState());
////            _motorLift.setPower(-.3);
////            Thread.sleep(5);
////            idle();
////        }
////
////        _motorLift.setPower(0);
//
//        RobotLog.d("OctobotAutonomousBlueSide::A");
//
//        resetAllDriveMotorEncoders();
//        resetNonDriveMotorEncoders();
//
//        runUsingEncoders();
//        runNonDriveUsingEncoders();
//
////       int initialLift =  _motorLift.getCurrentPosition();
////
////        RobotLog.d("OctobotAutonomousBlueSide::_motorLift.getCurrentPosition():: " + _motorLift.getCurrentPosition());
//        RobotLog.d("OctobotAutonomousBlueSide::B");
//
//        _servoLock.setPosition(.5);
//
//        grabberServo.setPosition(0);
//
//        RobotLog.d("OctobotAutonomousBlueSide::C");
//
//        telemetry.addLine("Waiting for start");
//        telemetry.update();
//
//        waitForStart();
//
//        RobotLog.d("OctobotAutonomousBlueSide::D");
//
//        telemetry.addLine("Ready");
//        telemetry.update();
//
//        RobotLog.d("OctobotAutonomousBlueSide::E");
//
//        RobotLog.d("OctobotAutonomousBlueSide::H");
//
//        // Drive for 48 inches at .5 powe
//
//        MotorRunner liftRaiser = new MotorRunner(_motorLift, 1, 0, new TimeStopper(900, null));
//        liftRaiser.startMotor();
//
//        jewel(initialHeading, false);
//
//        MotorRunner slideRunner = new MotorRunner(_motorSlide, -1, 0, new DigitalChannelStopper(_rearSliderLimit, true, 4000, null));
//        slideRunner.startMotor();
//
//        stopVuforia();
//
//    }
//
//}
