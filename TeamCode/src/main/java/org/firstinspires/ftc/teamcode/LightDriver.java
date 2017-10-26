package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/17/2017.
 */
public class LightDriver extends DefaultDriver {
    double _power;
    AnalogInput _analogInput;
    public DigitalChannel _led;

    int _maxPosition;

    double _minVoltageMeasured;
    double _maxVoltageMeasured;

    double _strength;

    double _minVoltage;
    double _maxVoltage;

    public LightDriver(double power, AnalogInput analogInput, DigitalChannel led,
                       double minVoltage, double maxVoltage, double strength, int maxPosition, DriverIF driver) {
        super(driver);

        _power = power;

        _analogInput = analogInput;
        _led = led;

        _minVoltage = minVoltage;
        _maxVoltage = maxVoltage;

        _strength = strength;
        _maxPosition = maxPosition;
    }

    @Override
    public void start() {
        _led.setState(true);

        _minVoltageMeasured = Double.MAX_VALUE;
        _maxVoltageMeasured = -Double.MAX_VALUE;
    }

    @Override
    public Steerage getSteerage() {
        double voltage = _analogInput.getVoltage();

        RobotLog.d("LightDriver::getSteeringFactor::voltage: " + voltage);

        _minVoltageMeasured = Math.min(_minVoltageMeasured, voltage);
        _maxVoltageMeasured = Math.max(_maxVoltageMeasured, voltage);

        return Steerage.createPowerSteerage(_power, _strength * 2.0 * ((voltage - _minVoltage) / (_maxVoltage - _minVoltage) - .5), 0);
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            keepGoing = position < _maxPosition;
        }

        return keepGoing;
    }

    @Override
    public void finish() {
        _led.setState(false);
    }

    double getMinVoltageMeasured() {
        return _minVoltageMeasured;
    }

    double getMaxVoltageMeasured() {
        return _maxVoltageMeasured;
    }
}
