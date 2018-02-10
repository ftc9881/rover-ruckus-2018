package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Keeps track of button position as if it were a toggle
 * Created by ftc on 2/5/2018.
 */

class ButtonManager {
    boolean _state;
    private boolean _changed;

    public void update(boolean newState) {
        _changed = (_state ^ newState);
        _state = newState;
    }

    public boolean isChanged() {
        return _changed;
    }

    public boolean getState() {
        return _state;
    }
}
