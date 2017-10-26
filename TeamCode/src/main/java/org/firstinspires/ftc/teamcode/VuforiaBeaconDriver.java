package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by ftc on 2/17/2017.
 */
public class VuforiaBeaconDriver extends DefaultDriver {
    private final VuforiaTrackableDefaultListener _listener;

    double _power;
    double _strength;

    double _angleThreshold;
    double _strafeThreshold;

    double _minDistance;
    double _distanceThreshold;

    private final int _maxPosition;

    public VuforiaBeaconDriver(VuforiaTrackableDefaultListener listener, double power, double strength,
                               double angleThreshold, double strafeThreshold, double minDistance, double distanceThreshold,
                               int maxPosition, DriverIF driver) {
        super(driver);
        _listener = listener;

        _power = power;
        _strength = strength;

        _angleThreshold = angleThreshold;
        _strafeThreshold = strafeThreshold;

        _minDistance = minDistance;
        _distanceThreshold = distanceThreshold;

        _maxPosition = maxPosition;
    }

    @Override
    public Steerage getSteerage() {
        OpenGLMatrix rawPose = _listener.getRawPose();

        if(rawPose != null) {
            VectorF angles = VuforiaUtil.anglesFromTarget(rawPose);
            VectorF translation = _listener.getPose().getTranslation();

                /*
                    Z is distance away
                    Y is left and right
                */

            double theta = angles.get(1);
            double offset = translation.get(1);
            double distance = translation.get(2);

            RobotLog.d("VuforiaBeaconDriver::getSteerage::theta: " + theta);
            RobotLog.d("VuforiaBeaconDriver::getSteerage::offset: " + offset);
            RobotLog.d("VuforiaBeaconDriver::getSteerage::distance: " + distance);

            double strafePower;
            double forwardPower;

            if(Math.abs(offset) > _strafeThreshold * 10) {
                strafePower = _power * (offset > 0 ? -1 : 1);
            }
            else if(Math.abs(offset) > _strafeThreshold) {
                strafePower = _power * (.1 + .9 * Math.abs(offset) / (_strafeThreshold * 10)) * (offset > 0 ? -1 : 1);
            }
            else {
                strafePower = 0;
            }

            double distanceDiff = distance - _minDistance;

            if(Math.abs(distanceDiff) > _distanceThreshold * 10) {
                forwardPower = _power * (distanceDiff > 0 ? 1 : -1);
            }
            else if(Math.abs(distanceDiff) > _distanceThreshold) {
                forwardPower = _power * (.1 + .9 * Math.abs(distanceDiff) / (_distanceThreshold * 10)) * (distanceDiff > 0 ? 1 : -1);
            }
            else {
                forwardPower =  0;
            }

            double steeringFactor;

            if(Math.abs(theta) > 5) {
                steeringFactor = -_strength * (theta > 5 ? 5 : -5);
            }
            else if(Math.abs(theta) > _angleThreshold ) {
                steeringFactor = -_strength * theta;
            }
            else {
                steeringFactor = 0;
            }

            RobotLog.d("VuforiaBeaconDriver::getSteerage::strafePower: " + strafePower);
            RobotLog.d("VuforiaBeaconDriver::getSteerage::strafePower: " + forwardPower);
            RobotLog.d("VuforiaBeaconDriver::getSteerage::steeringFactor: " + steeringFactor);

            return new Steerage(forwardPower - _power * steeringFactor, forwardPower + _power * steeringFactor, strafePower );
        }
        else {
            RobotLog.d("VuforiaBeaconDriver::getSteerage::no pose");

            return Steerage.createStationary();
        }
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            keepGoing = position < _maxPosition;
        }

        if(keepGoing) {
            OpenGLMatrix rawPose = _listener.getRawPose();

            if(rawPose != null) {
                VectorF angles = VuforiaUtil.anglesFromTarget(rawPose);
                VectorF translation = _listener.getPose().getTranslation();

                double theta = angles.get(1);
                double offset = translation.get(1);
                double distance = translation.get(2);

                double distanceDiff = distance - _minDistance;

                keepGoing = Math.abs(theta) > _angleThreshold || Math.abs(offset) > _strafeThreshold || Math.abs(distanceDiff) > _distanceThreshold;
            }
        }
        return keepGoing;
    }
}
