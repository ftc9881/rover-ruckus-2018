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

public class IMUTurner implements TurnerIF {
    double _power;
    double _degrees;
    double _strength;
    BNO055IMU _imu;
    double _initialHeading;
    double _maxError;

    IMUTurner(double degrees, double power, BNO055IMU imu, double strength, double maxError) {
        _degrees = degrees;
        _power = power;
        _imu = imu;
        _strength = strength;
        _maxError = maxError;
    }

    @Override
    public void start() {
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
        RobotLog.d("IMUTurner::getScaleFactor()::A");

        Orientation angles = _imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

        RobotLog.d("IMUTurner::getScaleFactor()::angles " + angles);

        double absHeading = angles.firstAngle;

//        RobotLog.d("IMUTurner::getSteeringFactor::absHeading: " + absHeading);

        double heading = absHeading - _initialHeading;
        heading += (heading > 180) ? -360 : (heading < -180) ? 360 : 0;

//        RobotLog.d("IMUTurner::getSteeringFactor::heading: " + heading);

        double difference = _degrees - heading;
        difference += (difference > 180) ? -360 : (difference < -180) ? 360 : 0;

        RobotLog.d("IMUTurner::getScaleFactor()::difference " + difference);

//        RobotLog.d("IMUTurner::getSteeringFactor::difference: " + difference);

        double effectiveDegrees = _degrees;

        if(Math.abs(effectiveDegrees) > 45) {
            effectiveDegrees = 45 * (_degrees > 0 ? 1 : -1);
        }

        RobotLog.d("IMUTurner::getScaleFactor()::effectiveDegrees " + effectiveDegrees);

        RobotLog.d("IMUTurner::getScaleFactor()::B");

        if(Math.abs(difference) < _maxError) {
            RobotLog.d("IMUTurner::getScaleFactor()::C");

            return Double.NaN;
        }
        else {
            RobotLog.d("IMUTurner::getScaleFactor()::D");

            double direction = difference > 0 ? 1 : -1;

            double fraction = Math.abs(difference / effectiveDegrees);

            RobotLog.d("IMUTurner::getScaleFactor()::fraction " + fraction);

//            RobotLog.d("IMUTurner::getSteeringFactor::fraction: " + fraction);

            double factor;

            if(fraction > .75) {
                factor = 1;
            }
            else if(fraction < .25) {
                factor = _strength * (1 + fraction / .25) / 2;
            }
            else {
                factor = _strength + (1 - _strength) * (fraction - .25) * 2;
            }

            RobotLog.d("IMUTurner::getSteeringFactor()::direction: " + direction);
            RobotLog.d("IMUTurner::getSteeringFactor()::factor: " + factor);

            return direction * factor;
        }
    }

    @Override
    public void finish() {
    }

}
