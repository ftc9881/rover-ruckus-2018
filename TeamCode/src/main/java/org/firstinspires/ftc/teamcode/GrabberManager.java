package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

/**
 * Manages the grabber servo and its state based on button presses
 * Created by ftc on 2/5/2018.
 */

class GrabberManager {
    Servo _servo;
    int _positionIndex = 0;
    List<Number> _positions = null;
    boolean _fastClose;

    ButtonManager _openButtonManager = new ButtonManager();
    ButtonManager _closeButtonManager = new ButtonManager();

    GrabberManager(Servo servo, List<Number> positions, boolean fastClose) {
        _servo = servo;
        _positions = positions;
        _fastClose = fastClose;
    }

    public void updateState(boolean buttonOpen, boolean buttonClose) {
        _openButtonManager.update(buttonOpen);
        _closeButtonManager.update(buttonClose);

        if(_openButtonManager.getState() && _openButtonManager.isChanged() && _positionIndex < _positions.size() - 1) {
            ++_positionIndex;
        }

        if(_closeButtonManager.getState() && _closeButtonManager.isChanged() && _positionIndex > 0) {
            if(_fastClose) {
                _positionIndex = 0;
            }
            else {
                --_positionIndex;
            }
        }

        double newPosition = _positions.get(_positionIndex).doubleValue();

//        RobotLog.d("GrabberManager::updateState()::newPosition: " + newPosition);

        _servo.setPosition(newPosition);
    }

    public void setPositionIndex(int positionIndex) {
        _positionIndex = positionIndex;

        double newPosition = _positions.get(positionIndex).doubleValue();

        RobotLog.d("GrabberManager::setPositionIndex()::newPosition: " + newPosition);

        _servo.setPosition(newPosition);
    }
}

