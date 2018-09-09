//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.LEFT;
//
///*
// * An example linear op mode where the pushbot
// * will drive in a square pattern using sleep()
// * and a for loop.
// */
//@Autonomous(name = "TestReturn", group = "test")
//public class TestReturn extends OctobotMain
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
//        Servo otherGrabber = getGrabberServo();
//
//        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.LEFT;
//
//        grabCubeFromPileIR(initialHeading, false, otherGrabber, vuMark);
//
//    }
//
//}
