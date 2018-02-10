package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "OctobotAutonomousRedCorner", group = "octobot")
public class OctobotAutonomousRedCorner extends OctobotMain
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
        RobotLog.d("OctobotAutonomousRedCorner::B");

        RobotLog.d("OctobotAutonomousRedCorner::C");

        telemetry.addLine("Waiting for start");
        telemetry.update();

        _servoLock.setPosition(.5);

        grabberServo.setPosition(0);

        waitForStart();

        RelicRecoveryVuMark vuMark = getVuforia();

        RobotLog.d("OctobotAutonomousRedCorner::VuMark::"+vuMark);

        RobotLog.d("OctobotAutonomousRedCorner::D");

        telemetry.addLine("Ready");
        telemetry.update();

        RobotLog.d("OctobotAutonomousRedCorner::E");

        MotorRunner liftRaiser = new MotorRunner(_motorLift, 1, new TimeStopper(900, null));
        liftRaiser.startMotor();

        jewel(initialHeading, true);

        // Open slide once we're off the balancing stone

        MotorRunner slideRunner = new MotorRunner(_motorSlide, -1, new DigitalChannelStopper(_button1, false, 4000, null));
        slideRunner.startMotor();

        // Drive to crypto box

        drive(new IMUDriver(.6, 0, _imu1, .04, initialHeading, RobotControl.convertInches(24), null), true, true);

        if (vuMark == RelicRecoveryVuMark.LEFT){
            drive(new IMUDriver(.6, 0, _imu1, .04, initialHeading, RobotControl.convertInches(16), null), true, true);
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER){
            drive(new IMUDriver(.6, 0, _imu1, .04, initialHeading, RobotControl.convertInches(8.5f), null), true, true);
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT){
        }
        else{
            drive(new IMUDriver(.6, 0, _imu1, .04, initialHeading, RobotControl.convertInches(8.5f), null), true, true);
        }

        doTheWholeThing(initialHeading, grabberServo, vuMark, true);

        stopVuforia();
    }

}
