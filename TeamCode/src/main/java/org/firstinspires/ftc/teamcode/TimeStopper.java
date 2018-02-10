package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/6/2018.
 */

public class TimeStopper extends DefaultStopper {
    protected long _maxTime = Long.MAX_VALUE;

    protected long _startTime;

    public TimeStopper(long maxTime, StopperIF stopper) {
        super(stopper);

        _maxTime = maxTime;
    }

    public void start() {
        super.start();

        _startTime = System.currentTimeMillis();
    }

    public boolean keepGoing(int position) {
        if(super.keepGoing(position)) {
            return (System.currentTimeMillis() - _startTime) < _maxTime;
        }
        else {
            return false;
        }
    }

    public void finish() {
        super.finish();
    }
}
