package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name = "OctobotCubeFinder", group = "octobot")
public class OctobotCubeFinder extends OctobotMain
{
    class DistanceLogger extends DefaultDriver {
        DistanceSensorIF _distanceSensor;

        public DistanceLogger(DistanceSensorIF distanceSensor, DriverIF driver) {
            super(driver);
            _distanceSensor = distanceSensor;
        }

        public boolean keepGoing(int position) {
            RobotLog.d("DistanceLogger::distance::" + _distanceSensor.getDistance());
            return super.keepGoing(position);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        DistanceLogger distanceLogger = new DistanceLogger(_irSensorLeft, null);

        drive(new StrafeDriver(.25f, RobotControl.convertInchesStrafe(23), distanceLogger));
    }

}
