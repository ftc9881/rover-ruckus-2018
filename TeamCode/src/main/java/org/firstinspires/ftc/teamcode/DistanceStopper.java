package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/19/2017.
 */
public class DistanceStopper extends DefaultDriver {
    private DistanceSensorIF _distanceSensor;
    private final double _minDistance;
    private final double _maxDistance;

    public DistanceStopper(DistanceSensorIF distanceSensor, double minDistance, double maxDistance, DriverIF driver) {
        super(driver);
        _distanceSensor = distanceSensor;
        _minDistance = minDistance;
        _maxDistance = maxDistance;
    }


    @Override
    public void start() {
        super.start();
    }

    @Override
    public Steerage getSteerage() {
        Steerage steerage = super.getSteerage();

        if(steerage == null) {
            return Steerage.createStationary();
        }
        else {
            return steerage;
        }
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            double distance = _distanceSensor.getDistance();

            RobotLog.d("DistanceStopper::keepGoing:: " + distance);

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
