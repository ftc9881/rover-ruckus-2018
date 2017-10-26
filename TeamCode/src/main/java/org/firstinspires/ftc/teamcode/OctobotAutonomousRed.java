//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.RobotLog;
//
///*
// * An example linear op mode where the pushbot
// * will drive in a square pattern using sleep()
// * and a for loop.
// */
//@Autonomous(name = "OctobotAutonomousRed", group = "octobot")
//@Disabled
//public class OctobotAutonomousRed extends OctobotMain
//{
//
//    protected ModernRoboticsI2cGyro _gyro;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initialize();
//        loadCalibration();
//
//        telemetry.addData("analog0 Target", getTarget0());
//        telemetry.addData("analog1 Target", getTarget1());
//        telemetry.update();
//
//        RobotLog.d("analog0 Target " + getTarget0());
//        RobotLog.d("analog1 Target " + getTarget1());
//
//        waitForStart();
//
//        _servoRight.setPower(.5f);
//        _servoLeft.setPower(-.5f);
//        RobotControl.moveWithGyro(RobotControl.convertInches(2), 1f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100, null, _sensorRGB, this, false );
//        RobotControl.turnWithGyro(-37, .5f, _motorA, _motorB, _motorC, _motorD, _gyro, this, _analog0, 100);
//        _servoRight.setPower(0f);
//        _servoLeft.setPower(0f);
//
//        RobotControl.moveWithGyro(RobotControl.convertInches(25), 1f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100, null, _sensorRGB, this, false );
//
//        // Shoot the particles
//
//        Thread.sleep(300);
//        _motorF.setPower(1);
//        Thread.sleep(500);
//        _motorF.setPower(0);
//
//        // Position for strafing
//
//        RobotControl.moveWithGyro(RobotControl.convertInches(19), 1f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100, null, _sensorRGB, this, false );
//        RobotControl.turnWithGyro(-42, .5f, _motorA, _motorB, _motorC, _motorD, _gyro, this, _analog0, 100);
//
//        // Strafe until we get to the white line
//
//        RobotControl.moveWithGyro(RobotControl.convertInchesStrafe(10), -85f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100,  null, _sensorRGB, this, true );
//
//        _led0.setState(true);
//        RobotControl.moveWithGyro(RobotControl.convertInchesStrafe(12), -.25f, _motorA, _motorB, _motorC, _motorD, _gyro, _analog0, getTarget0(),  null, _sensorRGB, this, true );
//        _led0.setState(false);
//
//        _led1.setState(true);
//        RobotControl.moveWithGyro(RobotControl.convertInchesStrafe(1), -.05f, _motorA, _motorB, _motorC, _motorD, _gyro, _analog1, getTarget1(),  null, _sensorRGB, this, true );
//        _led1.setState(false);
//
//        // Push the left button
//
//        pushButton(getTarget1());
//
//        // Back up and move to next beacon
//
//        RobotControl.moveWithGyro(RobotControl.convertInches(6), -1f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100, null, _sensorRGB, this, false );
//        RobotControl.moveWithGyro(RobotControl.convertInchesStrafe(45), -.85f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100,  null, _sensorRGB, this, true );
//
//        _led0.setState(true);
//        RobotControl.moveWithGyro(RobotControl.convertInchesStrafe(15), -.25f, _motorA, _motorB, _motorC, _motorD, _gyro, _analog0, getTarget0(),  null, _sensorRGB, this, true );
//        _led0.setState(false);
//
//        _led1.setState(true);
//        RobotControl.moveWithGyro(RobotControl.convertInchesStrafe(1), -.05f, _motorA, _motorB, _motorC, _motorD, _gyro, _analog1, getTarget1(),  null, _sensorRGB, this, true );
//        _led1.setState(false);
//
//        pushButton(getTarget1());
//
//        RobotControl.moveWithGyro(RobotControl.convertInches(12), -1f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100, null, _sensorRGB, this, false );
//        RobotControl.turnWithGyro(43, .5f, _motorA, _motorB, _motorC, _motorD, _gyro, this, _analog0, 100);
//        RobotControl.moveWithGyro(RobotControl.convertInches(55), -1f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100, null, _sensorRGB, this, false );
//
////        RobotControl.moveWithGyro(RobotControl.convertInches(20), 1f, _motorA, _motorB, _motorC, _motorD, _gyro, _analog0, .05, this );
////        RobotControl.turnWithGyro(83,.25f, _motorA, _motorB, _motorC, _motorD, _gyro, this);
//    }
//
//    void pushButton(double target1) throws InterruptedException {
//
//        FtcI2cDeviceState ftcI2cDeviceState = new FtcI2cDeviceState(_gyro);
//
//        ftcI2cDeviceState.setEnabled(false);
//
//        ColorStopper stopper = new ColorStopper(_sensorRGB, 25,
//                new ButtonStopper(_pusherButton, null)
//        );
//
//        _led1.setState(true);
//
//        RobotControl.moveWithLight(RobotControl.convertInches(24), .15f, _motorA, _motorB, _motorC, _motorD, _analog1, target1, stopper, this, -.05);
//
//        _led1.setState(false);
//
//        ftcI2cDeviceState.setEnabled(true);
//
//        int deltaRed = _sensorRGB.redColor() - stopper.getAmbientRed();
//        int deltaBlue = _sensorRGB.blueColor() - stopper.getAmbientBlue();
//
//        if (deltaBlue > deltaRed && !_pusherButton.getState()) {
//            RobotControl.moveWithGyro(RobotControl.convertInchesStrafe(4.5f), .5f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100, null, _sensorRGB, this, true);
//        }
//
//        RobotControl.moveWithGyro(RobotControl.convertInches(18), .25f, _motorA, _motorB, _motorC, _motorD, _gyro, null, 100, _pusherButton, _sensorRGB, this, false);
//    }
//}
