package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/19/2017.
 */
public class DistanceSensorStopper extends DefaultStopper {
    private DistanceSensorIF _distanceSensor;
    private final double _minDistance;
    private final double _maxDistance;

    public DistanceSensorStopper(DistanceSensorIF distanceSensor, double minDistance, double maxDistance, StopperIF stopper) {
        super(stopper);
        _distanceSensor = distanceSensor;
        _minDistance = minDistance;
        _maxDistance = maxDistance;
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            double distance = _distanceSensor.getDistance();

            RobotLog.d("DistanceSensorStopper::keepGoing:: " + distance);

            if (distance < _minDistance || distance > _maxDistance) {
                keepGoing = false;
            }
        }

        return keepGoing;
    }

    @Override
    public void finish() {
        super.finish();
    }
}
