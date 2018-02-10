package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;

class CubeGrabberStopper extends DefaultStopper {
    DistanceSensorIF _distanceSensor;
    double lastDistance;
    double newDistance;
    double count;
    double difference;
    double totalDifference;
    double averageDifference;
    double absDifference;

    int _lastPosition;

    ArrayList<Double> _allDistances = new ArrayList<Double>();

    public CubeGrabberStopper(DistanceSensorIF distanceSensor, DriverIF driver) {
        super(driver);
        _distanceSensor = distanceSensor;
    }

    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        _lastPosition = position;

        double distance = _distanceSensor.getDistance();

        RobotLog.d("CubeGrabberStopper::position:: " + position);
        RobotLog.d("CubeGrabberStopper::distance:: " + distance);

        if(_allDistances.size() > 1) {
            int windowSize = 5;

            double sum = 0;

            int pointCount = Math.min(_allDistances.size(), windowSize);

            for (int i = 0; i < pointCount; ++i) {
                double value =_allDistances.get(_allDistances.size() - i - 1);
                sum += value;
            }

            double mean = sum / pointCount;

            double meanDiff = 0;
            double stdDevDiff = 1.5;

            if (_allDistances.size() > windowSize + 1) {
                double sumDiff = 0;
                double ssumDiff = 0;

                for (int i = 0; i < windowSize; ++i) {
                    double value =_allDistances.get(_allDistances.size() - i - 1);
                    double valuePrev =_allDistances.get(_allDistances.size() - i - 2);

                    double diff = value - valuePrev;
                    sumDiff += diff;
                    ssumDiff += diff * diff;
                }

                meanDiff = sumDiff / windowSize;
                stdDevDiff = Math.sqrt(ssumDiff / windowSize - Math.pow(meanDiff, 2));
            }

            double newDiff = distance - _allDistances.get(_allDistances.size() - 1);

            double zScore = (newDiff - meanDiff) / stdDevDiff;

            RobotLog.d("CubeGrabberStopper::distance mean meanDiff stdDevDiff newDiff zScore:: " + distance + " " + mean + " " + meanDiff + " " + stdDevDiff + " " + newDiff + " " + zScore);

            if (zScore > 2 && Math.abs(meanDiff) < 1 && mean < 40) {
                keepGoing = false;
            }
        }

        _allDistances.add(distance);

        return keepGoing;
    }

    public int getLastPosition() {
        return _lastPosition;
    }
}
