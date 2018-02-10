package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by ftc on 2/17/2017.
 */

public class IMUTurner extends DefaultStopper implements TurnerIF {
    double _power;
    double _degrees;
    BNO055IMU _imu;
    double _initialHeading;
    double _maxError;

    double _slowDifference = 25;
    double _rampDown = 3;
    double _minFactor = .15;

    IMUTurner(double degrees, double power, BNO055IMU imu, double maxError, StopperIF stopper) {
        super(stopper);

        _degrees = degrees;
        _power = power;
        _imu = imu;
        _maxError = maxError;
    }

    IMUTurner(double degrees, double power, BNO055IMU imu, double maxError, StopperIF stopper,
              double slowDifference, double rampDown, double minFactor) {
        this(degrees, power, imu, maxError, stopper);

        _slowDifference = slowDifference;
        _rampDown = rampDown;
        _minFactor = minFactor;
    }

    @Override
    public void start() {
        super.start();

        RobotLog.d("IMUTurner::start()::A");

        Orientation angles = _imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

        RobotLog.d("IMUTurner::angles::" + angles);

        _initialHeading = angles.firstAngle;

        RobotLog.d("IMUTurner::start()::B");
    }

    @Override
    public double getPower() {
        return _power;
    }

    @Override
    public double getScaleFactor() {
//        RobotLog.d("IMUTurner::getScaleFactor()::A");

        Orientation angles = _imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

//        RobotLog.d("IMUTurner::getScaleFactor()::angles " + angles);

        double absHeading = angles.firstAngle;

//        RobotLog.d("IMUTurner::getSteeringFactor::absHeading: " + absHeading);

        double heading = absHeading - _initialHeading;
        heading += (heading > 180) ? -360 : (heading < -180) ? 360 : 0;

//        RobotLog.d("IMUTurner::getSteeringFactor::heading: " + heading);

        double difference = _degrees - heading;
        difference += (difference > 180) ? -360 : (difference < -180) ? 360 : 0;

        RobotLog.d("IMUTurner::getScaleFactor()::difference " + difference);

        if(Math.abs(difference) < _maxError) {
//            RobotLog.d("IMUTurner::getScaleFactor()::C");

            return Double.NaN;
        }
        else {
//            RobotLog.d("IMUTurner::getScaleFactor()::D");

            double direction = difference > 0 ? 1 : -1;

            double factor;

            if(Math.abs(difference) > _slowDifference) {
                // Go full speed if we are more than 10 degrees different from target angle
                factor = 1;
            }
            else {
                factor = (1.0 - _minFactor) * Math.pow(Math.abs(difference) / _slowDifference, _rampDown) + _minFactor;
            }

//            RobotLog.d("IMUTurner::getSteeringFactor()::direction: " + direction);
//            RobotLog.d("IMUTurner::getSteeringFactor()::factor: " + factor);

            return direction * factor;
        }
    }
}
