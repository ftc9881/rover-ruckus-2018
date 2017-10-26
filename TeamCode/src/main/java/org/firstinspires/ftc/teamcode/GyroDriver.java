package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by ftc on 2/17/2017.
 */
public class GyroDriver extends DefaultDriver {
    double _power;
    ModernRoboticsI2cGyro _gyro;

    /**
       The P in PID
     */

    double _strength;
    int _maxPosition;

    public GyroDriver(double power, ModernRoboticsI2cGyro gyro, double strength, int maxPosition, DriverIF driver) {
        super(driver);

        _power = power;
        _gyro = gyro;
        _strength = strength;
        _maxPosition = maxPosition;
    }

    @Override
    public void start() {
        super.start();
        _gyro.resetZAxisIntegrator();
    }

    @Override
    public Steerage getSteerage() {
        int angleZ = _gyro.getIntegratedZValue();

        return Steerage.createPowerSteerage(_power, -angleZ * _strength, 0);
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            keepGoing = position < _maxPosition;
        }

        return keepGoing;
    }
}
