package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/6/2018.
 */

public class SaturationStopper extends DefaultStopper {
    NormalizedColorSensor _sensorRGB;
    float _minimumSaturation;
    float[] _hsvValues = new float[3];

    public SaturationStopper(NormalizedColorSensor sensorRGB, float minimumSaturation, StopperIF stopper) {
        super(stopper);
        _sensorRGB = sensorRGB;
        _minimumSaturation = minimumSaturation;
    }

    public void start() {
        super.start();
    }

    public boolean keepGoing(int position) {
        if(super.keepGoing(position)) {
            try {
                NormalizedRGBA colors = _sensorRGB.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), _hsvValues);

                RobotLog.d("SaturationStopper::keepGoing()::_hsvValues[1]: " + _hsvValues[1]);

                return _hsvValues[1] < _minimumSaturation;
            }
            catch(Exception e) {
                RobotLog.ee("SaturationStopper::keepGoing()::ColorSensor", e, e.getMessage());
            }

            return true;
        }
        else {
            return false;
        }
    }

    public void finish() {
        super.finish();
    }

    public float getHue() {
        return _hsvValues[0];
    }

    public float getSaturation() {
        return _hsvValues[1];
    }

    public float getValue() {
        return _hsvValues[2];
    }
}
