package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/19/2017.
 */
public class DefaultDriver implements DriverIF {
    private DriverIF _driver;

    public DefaultDriver(DriverIF driver) {
        _driver = driver;
    }

    public void setDriver(DriverIF driver) {
        _driver = driver;
    }

    @Override
    public void start() {
        if(_driver != null) {
            _driver.start();
        }
    }

    @Override
    public Steerage getSteerage() {
        if (_driver != null) {
            return _driver.getSteerage();
        }
        else {
            return null;
        }
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = true;

        if(_driver != null) {
            keepGoing = _driver.keepGoing(position);
        }

        return keepGoing;
    }

    @Override
    public void finish() {
        if(_driver != null) {
            _driver.finish();
        }
    }
}
