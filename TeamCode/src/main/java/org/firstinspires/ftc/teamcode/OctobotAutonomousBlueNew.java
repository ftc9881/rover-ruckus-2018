//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//
///*
// * An example linear op mode where the pushbot
// * will drive in a square pattern using sleep()
// * and a for loop.
// */
//@Autonomous(name = "OctobotAutonomousBlueNew", group = "octobot")
//public class OctobotAutonomousBlueNew extends OctobotMain
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
//        drive(new IMUDriver(0, .85, _imu, .04, initialHeading, RobotControl.convertInchesStrafe(23), null));
//
//        drive(new IMUDriver(1, 0, _imu, .06, initialHeading, RobotControl.convertInches(33), null));
//
//        VuforiaTrackableDefaultListener blueNearListener = getBeaconListener(BLUE_NEAR);
//
//        /*
//            Use light stopper or Vuforia to find line
//         */
//
//        drive(
//                new LightStopper(_analog1, _led1, getTarget1(),
//                    new VuforiaBeaconStopper(blueNearListener, 0, Double.NaN,
//                            new StrafeDriver(.5, RobotControl.convertInchesStrafe(20), null)
//                    )
//                )
//        );
//
//        if(!isBeaconVisible(blueNearListener)) {
//            drive(new LightStopper(_analog0, _led0, getTarget0(),
//                    new IMUDriver(0, .05, _imu, .04, initialHeading, RobotControl.convertInchesStrafe(1), null)
//                )
//            );
//        }
//
//        pushButton(true, blueNearListener, _analog0, _led0, _calibrationData._analog0Min, _calibrationData._analog0Max);
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
//        drive(new IMUDriver(0, .85, _imu, .04, initialHeading, RobotControl.convertInchesStrafe(35), null));
//
//        /*
//            Use light stopper or Vuforia to find line
//         */
//
//        VuforiaTrackableDefaultListener blueFarListener = getBeaconListener(BLUE_FAR);
//
//        RobotLog.d("OctobotAutonomousBlue::Driving to second beacon");
//
//        drive(
//                new LightStopper(_analog1, _led1, getTarget1(),
//                        new VuforiaBeaconStopper(blueFarListener, 0, Double.NaN,
//                                new StrafeDriver(.5, RobotControl.convertInchesStrafe(12), null)
//                        )
//                )
//        );
//
//        if(!isBeaconVisible(blueFarListener)) {
//            RobotLog.d("OctobotAutonomousBlue::refining on light sensor 0");
//            drive(new LightStopper(_analog0, _led0, getTarget0(), new StrafeDriver(.05, RobotControl.convertInchesStrafe(1), null)));
//        }
//
//        pushButton(true, blueFarListener, _analog0, _led0, _calibrationData._analog0Min, _calibrationData._analog0Max);
//
//        drive(new IMUDriver(-1, 0, _imu, .05, initialHeading, RobotControl.convertInches(4), null));
//
//        double currentHeading = getCurrentHeading();
//
//        double difference = currentHeading - initialHeading;
//        difference += (difference > 180) ? -360 : (difference < -180) ? 360 : 0;
//
//        RobotLog.d("OctobotAutonomousBlue::difference::" + difference);
//
//        turn(new IMUTurner(-60 - difference, .5, _imu, .2, 2));
//        drive(new IMUDriver(-1, 0, _imu, .05, Double.NaN, RobotControl.convertInches(65), null));
//
//        stopVuforia();
//    }
//
//}
