package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 2/6/2018.
 */

class DefaultStopper implements StopperIF {
    StopperIF _stopper = null;

    DefaultStopper(StopperIF stopper) {
        _stopper = stopper;
    }

    @Override
    public void start() {
        if(_stopper != null) {
            _stopper.start();
        }
    }

    @Override
    public boolean keepGoing(int position) {
        if(_stopper != null) {
            return _stopper.keepGoing(position);
        }
        else {
            return true;
        }
    }

    @Override
    public void finish() {
        if(_stopper != null) {
            _stopper.finish();
        }
    }
}
