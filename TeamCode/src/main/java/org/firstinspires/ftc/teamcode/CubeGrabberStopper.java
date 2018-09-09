package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;

import static java.lang.Thread.sleep;

class CubeGrabberStopper extends DefaultStopper implements Runnable {
    DistanceSensorIF _distanceSensorLeading;
    DistanceSensorIF _distanceSensorTrailing;
    private boolean _useThread;
    private int _sampleRate;
    private boolean _keepScanning;

    ArrayList<Double> _allDistancesTrailing = new ArrayList<Double>();
    ArrayList<Double> _allDistancesLeading = new ArrayList<Double>();

    ArrayList<Integer> _allPositions = new ArrayList<Integer>();

    private Thread _thread;
    private boolean _edgeDetectedTrailing;
    private boolean _edgeDetectedLeading;

    public CubeGrabberStopper(DistanceSensorIF distanceSensorLeading, DistanceSensorIF distanceSensorTrailing, StopperIF stopper, boolean useThread, int sampleRate) {
        super(stopper);
        _distanceSensorLeading = distanceSensorLeading;
        _distanceSensorTrailing = distanceSensorTrailing;
        _useThread = useThread;
        _sampleRate = sampleRate;
    }

    @Override
    public void start() {
        RobotLog.d("CubeGrabberStopper::start()::A");
        if(_useThread) {
            RobotLog.d("CubeGrabberStopper::start()::B");
            if(_thread == null) {
                RobotLog.d("CubeGrabberStopper::start()::C");
                _thread = new Thread(this);
            }

            RobotLog.d("CubeGrabberStopper::start()::D");

            _keepScanning = true;
            _thread.start();
            RobotLog.d("CubeGrabberStopper::start()::E");
        }
        RobotLog.d("CubeGrabberStopper::start()::F");
    }

    @Override
    public void run() {
        RobotLog.d("CubeGrabberStopper::run()::A");

        do {
            try {
                synchronized(_allDistancesLeading) {
                    double distance = getSensorDistanceLeading();
                    RobotLog.d("CubeGrabberStopper::run()::distance: " + distance);
                    _allDistancesLeading.add(distance);
                }

                synchronized(_allDistancesTrailing) {
                    double distance = getSensorDistanceTrailing();
                    RobotLog.d("CubeGrabberStopper::run()::distance: " + distance);
                    _allDistancesLeading.add(distance);
                }

                sleep(_sampleRate);
            } catch (InterruptedException e) {
            }

        } while (_keepScanning);

        RobotLog.d("CubeGrabberStopper::run()::B");
    }

    @Override
    public void finish() {
        _keepScanning = false;
    }

    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        _allPositions.add(position);

        RobotLog.d("CubeGrabberStopper::keepGoing()::position: " + position);

        RobotLog.d("CubeGrabberStopper::keepGoing()::_allPositions: " + _allPositions);

        if(!_useThread) {
//            RobotLog.d("CubeGrabberStopper::keepGoing()::A");

            double sensorDistanceTrailing = getSensorDistanceTrailing();
            RobotLog.d("CubeGrabberStopper::keepGoing()::sensorDistanceTrailing: " + sensorDistanceTrailing);
            _allDistancesTrailing.add(sensorDistanceTrailing);

            double sensorDistanceLeading = getSensorDistanceLeading();
            RobotLog.d("CubeGrabberStopper::keepGoing()::sensorDistanceLeading: " + sensorDistanceLeading);
            _allDistancesLeading.add(sensorDistanceLeading);
        }

//        RobotLog.d("CubeGrabberStopper::keepGoing()::B");

        synchronized(_allDistancesTrailing) {
            _edgeDetectedTrailing = detectEdge(_allDistancesTrailing, _allPositions, false, 5, 3, 50);
        }

        synchronized(_allDistancesLeading) {
            _edgeDetectedLeading = detectEdge(_allDistancesLeading, _allPositions, true, 5, 3, 50);
        }

        return !_edgeDetectedLeading && !_edgeDetectedTrailing;
    }

    private boolean detectEdge(ArrayList<Double> distanceList, ArrayList<Integer> positionList, boolean isLeading, int windowSize, double minZScore, int maxDistance) {
        //            RobotLog.d("CubeGrabberStopper::keepGoing()::position: " + position);

        boolean edgeDetected = false;

        if (distanceList.size() > windowSize + 2) {
            double distances[] = new double[2 + windowSize];
            double positions[] = new double[2 + windowSize];

            for (int i = 0; i < windowSize + 2; ++i) {
                int position = positionList.get(positionList.size() - i - 1);
                positions[i] = RobotControl.convertInchesStrafeInv(position) * 2.54;
            }

            if(isLeading) {
                for (int i = 0; i < windowSize + 2; ++i) {
                    distances[i] = distanceList.get(distanceList.size() - i - 1);
                }
            }
            else {
                for (int i = 0; i < windowSize + 2; ++i) {
                    distances[i] = distanceList.get(distanceList.size() - windowSize - 2 + i);
                }
            }

            double distance = distances[0];

//                RobotLog.d("CubeGrabberStopper::keepGoing()::distance: " + distance);

            double sum = 0;

            for (int i = 1; i < windowSize + 2; ++i) {
                double value = distances[i];
                sum += value;
            }

            double mean = sum / (windowSize + 1);

//                RobotLog.d("CubeGrabberStopper::keepGoing()::mean: " + mean);

            double sumDiff = 0;
            double ssumDiff = 0;
//            double sumSlope = 0;
            double sumPosDiff = 0;

            for (int i = 0; i < windowSize; ++i) {
                double value = distances[i + 1];
                double valuePrev = distances[i + 2];

                double diff = value - valuePrev;
                double posDiff = positions[i + 1] - positions[i + 2];

                RobotLog.d("CubeGrabberStopper::detectEdge()::posDiff: " + posDiff);
                RobotLog.d("CubeGrabberStopper::detectEdge()::diff: " + diff);

//                double slope = diff / posDiff;

//                RobotLog.d("CubeGrabberStopper::detectEdge()::slope: " + slope);

//                RobotLog.d("CubeGrabberStopper::detectEdge()::diff: " + diff);
                sumDiff += diff;
                ssumDiff += diff * diff;

//                sumSlope += slope;
                sumPosDiff += posDiff;
            }

            double meanDiff = sumDiff / windowSize;
            double stdDevDiff = Math.sqrt(ssumDiff / windowSize - Math.pow(meanDiff, 2));
            double meanSlope = sumDiff / sumPosDiff;

            double prevDistance = distances[1];

            double newDiff =  distance - prevDistance;

            double zScore = (newDiff - meanDiff) / stdDevDiff;

            RobotLog.d("CubeGrabberStopper::detectEdge()::leading distance mean meanSlope meanDiff stdDevDiff newDiff zScore:: " +
                    isLeading + " " + distance + " " + mean + " " + meanSlope + " " + meanDiff + " " + stdDevDiff + " " + newDiff + " " + zScore);

            if (newDiff > 2 && zScore > minZScore && mean < maxDistance && Math.abs(meanSlope) < 1) {
                edgeDetected = true;
            }
        }

        return edgeDetected;
    }

    private double getSensorDistanceLeading() {
        if(_distanceSensorLeading != null) {
            return _distanceSensorLeading.getDistance();
        }
        else {
            return Double.MAX_VALUE;
        }
    }

    private double getSensorDistanceTrailing() {
        if(_distanceSensorTrailing != null) {
            return _distanceSensorTrailing.getDistance();
        }
        else {
            return Double.MAX_VALUE;
        }
    }

    public int getLastPosition() {
        return _allPositions.get(_allPositions.size() - 1);
    }

    public boolean isEdgeDetectedLeading() {
        return _edgeDetectedLeading;
    }

    public boolean isEdgeDetectedTrailing() {
        return _edgeDetectedTrailing;
    }

}
