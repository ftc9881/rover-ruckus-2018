package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "OctobotAutonomousBlueCorner", group = "octobot")
public class OctobotAutonomousBlueCorner extends OctobotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        telemetry.addLine("Initializing VuForia");
        telemetry.update();

        initializeVuforia();
        startVuforia();

        double initialHeading = getCurrentHeading();

        Servo _servo;

        _servo = getGrabberServo();

//        while (!_button3.getState()){
//            RobotLog.d("OctobotAutonomousBlueCorner::_button3.getState()::" + _button3.getState());
//            _motorLift.setPower(-.3);
//            Thread.sleep(5);
//            idle();
//        }
//
//        _motorLift.setPower(0);

        RobotLog.d("OctobotAutonomousBlueCorner::A");

        resetAllDriveMotorEncoders();
        resetNonDriveMotorEncoders();

        runUsingEncoders();
        runNonDriveUsingEncoders();


//       int initialLift =  _motorLift.getCurrentPosition();
//
//        RobotLog.d("OctobotAutonomousBlueCorner::_motorLift.getCurrentPosition():: " + _motorLift.getCurrentPosition());
        RobotLog.d("OctobotAutonomousBlueCorner::B");

        RobotLog.d("OctobotAutonomousBlueCorner::C");

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        _servoLock.setPosition(.5);

        _servo.setPosition(1);

        RelicRecoveryVuMark vuMark = getVuforia();

        RobotLog.d("OctobotAutonomousBlueCorner::VuMark::"+vuMark);

        RobotLog.d("OctobotAutonomousBlueCorner::D");

        telemetry.addLine("Ready");
        telemetry.update();

        RobotLog.d("OctobotAutonomousBlueCorner::E");

        jewel(initialHeading, true);

        RobotLog.d("OctobotAutonomousBlueCorner::F");

        raiseLift1500();

        RobotLog.d("OctobotAutonomousBlueCorner::H");

        // Drive for 48 inches at .5 powe

        drive(new IMUDriver(-.7, 0, _imu1, .04, initialHeading, RobotControl.convertInches(30), null));

        RobotLog.d("OctobotAutonomousBlueCorner::I");

        // Strafe for 23 inches at .25 power

//        double targetHeading = getCurrentHeading() + 90;
//        targetHeading += (targetHeading > 180) ? -360 : (targetHeading < -180) ? 360 : 0;
//
//        double difference = targetHeading - initialHeading;
//        difference += (difference > 180) ? -360 : (difference < -180) ? 360 : 0;

        RobotLog.d("OctobotAutonomousBlueCorner::J");

        turn(new IMUTurner(-(getCurrentHeading() - initialHeading), .1, _imu1, .2, .25));

        RobotLog.d("OctobotAutonomousBlueCorner::K");

        if (vuMark == RelicRecoveryVuMark.LEFT){
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER){
            drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInches(7), null));
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT){
            drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInches(14), null));
        }
        else{
            drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInches(7), null));
        }

        turn(new IMUTurner(-(getCurrentHeading() - initialHeading) - 90, .3, _imu1, .2, 1));

        //if blue, true; if red, false

        deliverBlockCorner(initialHeading, _servo, true);

        afterBlockCorner(initialHeading, true);

        RobotLog.d("OctobotAutonomousBlueCorner::L");

        stopVuforia();

    }

}
