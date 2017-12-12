package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 12/6/2017.
 */

public class ButtonModeSwitcher {
    long _lastPressedTime = 0;
    boolean _mode = false;
    boolean _isLastPressed;

    public ButtonModeSwitcher(boolean initialMode) {
        _mode = initialMode;
    }

    public void changeMode(boolean isPressed) {
        long currentTimeMillis = System.currentTimeMillis();

        if(isPressed ^ _isLastPressed && currentTimeMillis > _lastPressedTime) {
            _lastPressedTime = currentTimeMillis;
            _isLastPressed = isPressed;

            _mode = !_mode;
        }
    }

    public boolean getMode() {
        return _mode;
    }
}
