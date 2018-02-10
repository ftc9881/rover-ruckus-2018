package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by ftc on 2/6/2018.
 */

public class DigitalChannelStopper extends TimeStopper {
    private DigitalChannel _digitalChannel;
    private boolean _keepGoingState;

    protected long _startTime;

    public DigitalChannelStopper(DigitalChannel digitalChannel, boolean keepGoingState, long maxTime, StopperIF stopper) {
        super(maxTime, stopper);
        _digitalChannel = digitalChannel;
        _keepGoingState = keepGoingState;
    }

    public void start() {
        super.start();
    }

    public boolean keepGoing(int position) {
        if(super.keepGoing(position)) {
            return _digitalChannel.getState() == _keepGoingState;
        }
        else {
            return false;
        }
    }

    public void finish() {
        super.finish();
    }
}
