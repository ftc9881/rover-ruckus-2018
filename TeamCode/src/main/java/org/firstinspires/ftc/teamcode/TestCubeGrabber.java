//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//
//@TeleOp(name = "TestCubeGrabber", group = "test")
//public class TestCubeGrabber extends OctobotMain
//{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addLine("Initializing Motors and Sensors");
//        telemetry.update();
//
//        initialize();
//
//        waitForStart();
//
//        double initialHeading = getCurrentHeading();
//
//        RobotLog.d("TestCubeGrabber::initialHeading::" + initialHeading);
//
//        Servo grabberServo = getGrabberServo();
//
//        Servo otherGrabber;
//
//        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.LEFT;
//
//        if (grabberServo == _servoGrabberBlue) {
//            otherGrabber = _servoGrabberRed;
//        } else if (grabberServo == _servoGrabberRed) {
//            otherGrabber = _servoGrabberBlue;
//        } else {
//            otherGrabber = null;
//        }
//
//        grabCubeFromPileIR(initialHeading + 90, true, otherGrabber, vuMark);
//
//    }
//
//}
