//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//
///*
// * An example linear op mode where the pushbot
// * will drive in a square pattern using sleep()
// * and a for loop.
// */
//@Autonomous(name = "OctobotAutonomousRedNew", group = "octobot")
//public class OctobotAutonomousRedNew extends OctobotMain
//{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addLine("Initializing Motors and Sensors");
//        telemetry.update();
//
//        initialize();
//
//        telemetry.addLine("Initializing Vuforia");
//        telemetry.update();
//
//        initializeVuforia();
//
//        telemetry.addLine("Calibrating");
//        telemetry.update();
//
//        loadCalibration();
//
//        telemetry.addLine("Ready");
//        telemetry.addLine("ORIENT ROBOT BACKWARD!");
//        telemetry.update();
//
//        waitForStart();
//
//        startVuforia();
//
//        /*
//            Get the initial heading of the robot
//         */
//
//        double initialHeading = getCurrentHeading();
//
//        RobotLog.d("OctobotAutonomousRed::initialHeading::" + initialHeading);
//
//        /*
//            Ensure tha arms are folded in
//         */
//
//        _servoRight.setPower(.5f);
//        _servoLeft.setPower(-.5f);
//
//       _servoRight.setPower(0f);
//       _servoLeft.setPower(0f);
//
//        drive(new IMUDriver(0, .85, _imu, .04, initialHeading, RobotControl.convertInchesStrafe(7), null));
//
//        shootTwoParticles();
//
//        /*
//            Do a 180
//         */
//
//        initialHeading += 180;
//        initialHeading += (initialHeading > 180) ? -360 : (initialHeading < -180) ? 360 : 0;
//
//        RobotLog.d("OctobotAutonomousRed::new initialHeading::" + initialHeading);
//
//        double currentHeading = getCurrentHeading();
//
//        double difference = currentHeading - initialHeading;
//        difference += (difference > 180) ? -360 : (difference < -180) ? 360 : 0;
//
//        RobotLog.d("OctobotAutonomousRed::difference::" + difference);
//
//        turn(new IMUTurner(difference, .6, _imu, .2, 2));
//
//        /*
//            Continue on with rest of program
//         */
//
//        drive(new IMUDriver(0, -.85, _imu, .04, initialHeading, RobotControl.convertInchesStrafe(23), null));
//
//        drive(new IMUDriver(1, 0, _imu, .06, initialHeading, RobotControl.convertInches(33), null));
//
//        VuforiaTrackableDefaultListener redNearListener = getBeaconListener(RED_NEAR);
//
//        /*
//            Use light stopper or Vuforia to find line
//         */
//
//        drive(
//                new LightStopper(_analog0, _led0, getTarget0(),
//                    new VuforiaBeaconStopper(redNearListener, 0, Double.NaN,
//                            new StrafeDriver(-.5, RobotControl.convertInchesStrafe(24), null)
//                    )
//                )
//        );
//
//        if(!isBeaconVisible(redNearListener)) {
//            drive(new LightStopper(_analog1, _led1, getTarget1(),
//                    new IMUDriver(0, -.05, _imu, .04, initialHeading, RobotControl.convertInchesStrafe(1), null)
//                )
//            );
//        }
//
//        pushButton(false, redNearListener, _analog1, _led1, _calibrationData._analog1Min, _calibrationData._analog1Max);
//
//        /*
//            TODO: When power is negative then should the IMU driver reverse the steering direction?
//         */
//
//        /*
//            Back up and move to next beacon
//        */
//
//        drive(new IMUDriver(-1, 0, _imu, .05, initialHeading, RobotControl.convertInches(11), null));
//
//        drive(new IMUDriver(0, -.85, _imu, .04, initialHeading, RobotControl.convertInchesStrafe(35), null));
//
//        /*
//            Use light stopper or Vuforia to find line
//         */
//
//        VuforiaTrackableDefaultListener redFarListener = getBeaconListener(RED_FAR);
//
//        RobotLog.d("OctobotAutonomousRed::Driving to second beacon");
//
//        drive(
//                new LightStopper(_analog0, _led0, getTarget0(),
//                        new VuforiaBeaconStopper(redFarListener, 0, Double.NaN,
//                                new StrafeDriver(-.5, RobotControl.convertInchesStrafe(24), null)
//                        )
//                )
//        );
//
//        if(!isBeaconVisible(redFarListener)) {
//            RobotLog.d("OctobotAutonomousRed::refining on light sensor 1");
//            drive(new LightStopper(_analog1, _led1, getTarget1(), new StrafeDriver(-.05, RobotControl.convertInchesStrafe(1), null)));
//        }
//
//        pushButton(false, redFarListener, _analog1, _led1, _calibrationData._analog1Min, _calibrationData._analog1Max);
//
//        drive(new IMUDriver(-1, 0, _imu, .05, initialHeading, RobotControl.convertInches(4), null));
//
//        currentHeading = getCurrentHeading();
//
//        difference = currentHeading - initialHeading;
//        difference += (difference > 180) ? -360 : (difference < -180) ? 360 : 0;
//
//        RobotLog.d("OctobotAutonomousRed::difference::" + difference);
//
//        turn(new IMUTurner(60 + difference, .5, _imu, .2, 2));
//        drive(new IMUDriver(-1, 0, _imu, .05, Double.NaN, RobotControl.convertInches(65), null));
//
//        stopVuforia();
//    }
//
//}
