package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 2/6/2018.
 */

public class PositionStopper extends DefaultStopper {
    private int _minPosition;
    private int _maxPosition;

    public PositionStopper(int minPosition, int maxPosition, StopperIF stopper) {
        super(stopper);
        _minPosition = minPosition;
        _maxPosition = maxPosition;
    }

    public void start() {
        super.start();
    }

    public boolean keepGoing(int position) {
        if(super.keepGoing(position)) {
            return position >= _minPosition && position <= _maxPosition;
        }
        else {
            return false;
        }
    }

    public void finish() {
        super.finish();
    }
}
