package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 2/19/2017.
 */
public class DistanceDriver implements DriverIF {
    private DriverIF _driver;
    double _power;
    int _maxPosition;

    public DistanceDriver(double power, int maxPosition, DriverIF driver) {
        _driver = driver;
        _power = power;
        _maxPosition = maxPosition;
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
            return Steerage.createPowerSteerage(_power, 0, 0);
        }
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = true;

        if (_driver != null) {
            keepGoing = _driver.keepGoing(position);
        }

        if(keepGoing) {
            keepGoing = position < _maxPosition;
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
