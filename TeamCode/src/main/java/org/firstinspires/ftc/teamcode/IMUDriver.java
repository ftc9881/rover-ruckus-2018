package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by ftc on 2/17/2017.
 */
public class IMUDriver extends DefaultDriver {
    double _power;
    double _strafePower;
    double _strength;
    BNO055IMU _imu;
    int _maxPosition;
    double _initialHeading;

    public IMUDriver(double power, double strafePower, BNO055IMU imu, double strength, double initialHeading, int maxPosition, StopperIF stopper) {
        super(stopper);

        _power = power;
        _strafePower = strafePower;
        _imu = imu;
        _strength = strength;
        _initialHeading = initialHeading;
        _maxPosition = maxPosition;
    }

    @Override
    public void start() {
        super.start();

        if(Double.isNaN(_initialHeading)) {
            Orientation angles = _imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
            _initialHeading = angles.firstAngle;
        }
    }

    @Override
    public Steerage getSteerage() {
        Orientation angles = _imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        double heading = angles.firstAngle;

//        RobotLog.d("IMUDriver::getSteeringFactor::heading: " + heading);

        double difference = heading - _initialHeading;
        difference += (difference > 180) ? -360 : (difference < -180) ? 360 : 0;

//        RobotLog.d("IMUDriver::getSteeringFactor::difference: " + difference);
//        RobotLog.d("IMUDriver::getSteeringFactor::_strength: " + _strength);
//        RobotLog.d("IMUDriver::getSteeringFactor::_strafePower: " + _strafePower);

        return Steerage.createPowerSteerage(_power, difference * _strength, _strafePower);
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if (keepGoing) {
            keepGoing = position < _maxPosition;
        }

        return keepGoing;
    }
}
