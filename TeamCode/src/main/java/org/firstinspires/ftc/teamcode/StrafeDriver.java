package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 2/17/2017.
 */

public class StrafeDriver extends DefaultDriver {
    double _power;
    int _maxPosition;

    StrafeDriver(double power, int maxPosition, DriverIF driver) {
        super(driver);

        _power = power;
        _maxPosition = maxPosition;
    }

    @Override
    public Steerage getSteerage() {
        return new Steerage(0, 0, _power);
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
