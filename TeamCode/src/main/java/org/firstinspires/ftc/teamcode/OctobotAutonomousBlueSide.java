package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "OctobotAutonomousBlueSide", group = "octobot")
@Disabled
public class OctobotAutonomousBlueSide extends OctobotMain
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

        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        Servo _servo = getGrabberServo();

        RobotLog.d("OctobotAutonomousBlueSide::initialHeading::" + initialHeading);

//        while (!_button3.getState()){
//            RobotLog.d("OctobotAutonomousBlueSide::_button3.getState()::" + _button3.getState());
//            _motorLift.setPower(-.3);
//            Thread.sleep(5);
//            idle();
//        }
//
//        _motorLift.setPower(0);

        RobotLog.d("OctobotAutonomousBlueSide::A");

        resetAllDriveMotorEncoders();
        resetNonDriveMotorEncoders();

        runUsingEncoders();
        runNonDriveUsingEncoders();

//       int initialLift =  _motorLift.getCurrentPosition();
//
//        RobotLog.d("OctobotAutonomousBlueSide::_motorLift.getCurrentPosition():: " + _motorLift.getCurrentPosition());
        RobotLog.d("OctobotAutonomousBlueSide::B");

        RobotLog.d("OctobotAutonomousBlueSide::C");

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        _servoLock.setPosition(.5);

        _servo.setPosition(1);

        RelicRecoveryVuMark vuMark = getVuforia();

        RobotLog.d("OctobotAutonomousBlueSide::VuMark::"+vuMark);

        RobotLog.d("OctobotAutonomousBlueSide::D");

        telemetry.addLine("Ready");
        telemetry.update();

        RobotLog.d("OctobotAutonomousBlueSide::E");

        jewel(initialHeading, true);

        RobotLog.d("OctobotAutonomousBlueSide::H");

        // Drive for 48 inches at .5 powe

        drive(new IMUDriver(.7, 0, _imu1, .04, initialHeading, RobotControl.convertInches(25), null), true, true);

        RobotLog.d("OctobotAutonomousBlueSide::I");

        // Strafe for 23 inches at .25 power

//        double targetHeading = getCurrentHeading() + 90;
//        targetHeading += (targetHeading > 180) ? -360 : (targetHeading < -180) ? 360 : 0;
//
//        double difference = targetHeading - initialHeading;
//        difference += (difference > 180) ? -360 : (difference < -180) ? 360 : 0;


        RobotLog.d("OctobotAutonomousBlueSide::J");

        RobotLog.d("OctobotAutonomousBlueSide::K");

        if (vuMark == RelicRecoveryVuMark.LEFT){
            drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(2), null), true, true);
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER){
            drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(8), null), true, true);
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT){
            drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(15), null), true, true);
        }
        else{
            drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(8), null), true, true);
        }

        deliverBlockSide(initialHeading, _servo, true);

        afterBlockSide(initialHeading, true);

        RobotLog.d("OctobotAutonomousBlueSide::L");

        stopVuforia();

    }

}
