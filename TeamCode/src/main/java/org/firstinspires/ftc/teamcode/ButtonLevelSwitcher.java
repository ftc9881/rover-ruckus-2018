package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 12/6/2017.
 */

public class ButtonLevelSwitcher {
    long _lastPressedTime = 0;
    int _level;
    int _numLevels;
    boolean _isDownPressed;
    boolean _isUpPressed;

    public ButtonLevelSwitcher(int numLevels, int initialLevel) {
        _numLevels = numLevels;
        _level = initialLevel;
    }

    public void changeMode(boolean isUpPressed, boolean isDownPressed) {
        long currentTimeMillis = System.currentTimeMillis();

        if(( (isDownPressed ^= _isDownPressed) || (isUpPressed ^= _isUpPressed)) && currentTimeMillis > _lastPressedTime) {
            _lastPressedTime = currentTimeMillis;

            if(isDownPressed ^= _isDownPressed) {
                _isDownPressed = isDownPressed;

                if (_level > 0) {
                    --_level;
                }
            }

            if(isDownPressed ^= _isDownPressed) {
                _isDownPressed = isDownPressed;

                if (_level < _numLevels - 1) {
                    ++_level;
                }
            }
        }
    }

    public int getLevel() {
        return _level;
    }
}
