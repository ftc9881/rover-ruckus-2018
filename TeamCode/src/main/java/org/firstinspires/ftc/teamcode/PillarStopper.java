package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/19/2017.
 */
public class PillarStopper extends DefaultStopper {
    private DistanceSensorIF _distanceSensor;
    private final double _minDistance;
    private final double _maxDistance;

    private boolean _foundPillar;
    private double _lastDistance;

    public PillarStopper(DistanceSensorIF distanceSensor, double minDistance, double maxDistance, StopperIF stopper) {
        super(stopper);
        _distanceSensor = distanceSensor;
        _minDistance = minDistance;
        _maxDistance = maxDistance;
    }


    @Override
    public void start() {
        super.start();

        _foundPillar = false;
        _lastDistance = Double.MAX_VALUE;
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        RobotLog.d("PillarStopper::keepGoing()::position: " + position);
        RobotLog.d("PillarStopper::keepGoing()::_distanceSensor.getDistance(): " + _distanceSensor.getDistance());

        if(keepGoing) {
            double distance = _distanceSensor.getDistance();

            RobotLog.d("PillarStopper::keepGoing()::distance: " + distance);
            RobotLog.d("PillarStopper::keepGoing()::_foundPillar: " + _foundPillar);

            if(_foundPillar) {
                if(distance > _lastDistance) {
                    keepGoing = false;
                }
            }
            else {
                if(distance < _minDistance) {
                    _foundPillar = true;
                }
            }

            _lastDistance = distance;

            RobotLog.d("PillarStopper::keepGoing()::keepGoing: " + keepGoing);
        }

        return keepGoing;
    }

    @Override
    public void finish() {
        super.finish();
    }
}
