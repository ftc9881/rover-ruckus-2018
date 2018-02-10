package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

        Servo grabberServo = getGrabberServo();

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
        runNonDriveWithoutEncoders();


//       int initialLift =  _motorLift.getCurrentPosition();
//
//        RobotLog.d("OctobotAutonomousBlueCorner::_motorLift.getCurrentPosition():: " + _motorLift.getCurrentPosition());
        RobotLog.d("OctobotAutonomousBlueCorner::B");

        RobotLog.d("OctobotAutonomousBlueCorner::C");

        telemetry.addLine("Waiting for start");
        telemetry.update();

        _servoLock.setPosition(.5);

        grabberServo.setPosition(0);

        waitForStart();

        RelicRecoveryVuMark vuMark = getVuforia();

        RobotLog.d("OctobotAutonomousBlueCorner::VuMark::"+vuMark);

        RobotLog.d("OctobotAutonomousBlueCorner::D");

        telemetry.addLine("Ready");
        telemetry.update();

        RobotLog.d("OctobotAutonomousBlueCorner::E");

        MotorRunner liftRaiser = new MotorRunner(_motorLift, 1, new TimeStopper(900, null));
        liftRaiser.startMotor();

        jewel(initialHeading, false);

        RobotLog.d("OctobotAutonomousBlueCorner::F");

        RobotLog.d("OctobotAutonomousBlueCorner::H");

        // Drive to crypto box

        drive(new IMUDriver(-.6, 0, _imu1, .04, initialHeading, RobotControl.convertInches(26), null), true, true);

        // Open slide once we're off the balancing stone

        MotorRunner slideRunner = new MotorRunner(_motorSlide, -1, new DigitalChannelStopper(_button1, false, 4000, null));
        slideRunner.startMotor();

        if (vuMark == RelicRecoveryVuMark.LEFT){
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER){
            drive(new IMUDriver(-.6, 0, _imu1, .04, initialHeading, RobotControl.convertInches(6.5f), null), true, true);
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT){
            drive(new IMUDriver(-.6, 0, _imu1, .04, initialHeading, RobotControl.convertInches(15), null), true, true);
        }
        else{
            drive(new IMUDriver(-.6, 0, _imu1, .04, initialHeading, RobotControl.convertInches(7), null), true, true);
        }

        doTheWholeThing(initialHeading, grabberServo, vuMark, false);

        stopVuforia();
    }



}
