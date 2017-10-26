package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/19/2017.
 */
public class LightStopper extends DefaultDriver {
    private final AnalogInput _analogInput;
    private final double _target;
    private final DigitalChannel _led;
    private double _voltageEWMA;

    public LightStopper(AnalogInput analogInput, DigitalChannel led, double target, DriverIF driver) {
        super(driver);

        _analogInput = analogInput;
        _led = led;
        _target = target;
    }


    @Override
    public void start() {
        super.start();

        if(_led != null) {
            _led.setState(true);
        }

        _voltageEWMA = Double.NaN;
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
            double voltage = _analogInput.getVoltage();

            if(Double.isNaN(_voltageEWMA)) {
                _voltageEWMA = voltage;
            }
            else {
                _voltageEWMA = _voltageEWMA * .9 + voltage * .1;
            }

            RobotLog.d("LightStopper::keepGoing:: " + _target + " " + voltage + " " + _voltageEWMA);

            if (voltage > _target) {
                keepGoing = false;
            }
        }

        return keepGoing;
    }

    @Override
    public void finish() {
        super.finish();

        if(_led != null) {
            _led.setState(false);
        }
    }
}
