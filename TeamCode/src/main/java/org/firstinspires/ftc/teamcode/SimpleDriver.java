package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 2/17/2017.
 */

public class SimpleDriver extends DefaultDriver {
    double _power;
    int _maxPosition;

    SimpleDriver(double power, int maxPosition, DriverIF driver) {
        super(driver);

        _power = power;
        _maxPosition = maxPosition;
    }

    @Override
    public Steerage getSteerage() {
        return new Steerage(_power);
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            keepGoing =  position < _maxPosition;
        }

        return keepGoing;
    }
}
