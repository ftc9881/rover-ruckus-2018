package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "SensorStrafeTest", group = "octobot")
@Disabled
public class SensorStrafeTest extends OctobotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        resetAllDriveMotorEncoders();
        resetNonDriveMotorEncoders();

        runUsingEncoders();
        runNonDriveUsingEncoders();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        double initialHeading = getCurrentHeading();

        drive(new IMUDriver(0, -.1, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(53),
                new DefaultDriver(null) {
                    public boolean keepGoing(int position) {
                        RobotLog.d("SensorStrafeTest::sensors::position: " + position);

                        RobotLog.d("SensorStrafeTest::sensors::irRight: " + _irSensorRight.getDistance());
                        RobotLog.d("SensorStrafeTest::sensors::irLeft: " + _irSensorLeft.getDistance());
//
//                        RobotLog.d("SensorStrafeTest::sensors::sonarRight: " + _sonarRight.getDistance());
//                        RobotLog.d("SensorStrafeTest::sensors::sonarLeft: " + _sonarLeft.getDistance());

                        return super.keepGoing(position);
                    }
                }
                )
                , true, true);


    }

}
