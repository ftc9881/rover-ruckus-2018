package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/6/2018.
 */

public class SpinnerStopper extends DefaultStopper {
    ColorSensor _sensorRGB;
    float _minimumSaturation;
    float[] _hsvValues = new float[3];
    boolean isBlue;
    int _count;
    int _minCount;

    public SpinnerStopper(ColorSensor sensorRGB, float minimumSaturation, StopperIF stopper, boolean servoBlue, int minCount) {
        super(stopper);
        _sensorRGB = sensorRGB;
        _minimumSaturation = minimumSaturation;
        isBlue = servoBlue;
        _minCount = minCount;
        _count = 0;

    }

    public void start() {
        super.start();
    }

    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);
        if (keepGoing) {
            try {
//                NormalizedRGBA colors = _sensorRGB.getNormalizedColors();
//                Color.colorToHSV(colors.toColor(), _hsvValues);
                Color.RGBToHSV((_sensorRGB.red() * 255) / 800, (_sensorRGB.green() * 255) / 800, (_sensorRGB.blue() * 255) / 800, _hsvValues);

//                RobotLog.d("SpinnerStopper::RGB::" + colors.red + " " + colors.green + " " + colors.blue);
                RobotLog.d("SpinnerStopper::HSV::" + _hsvValues[0] + " " + _hsvValues[1] + " " + _hsvValues[2]);

                float hue = _hsvValues[0];
                float saturation = _hsvValues[1];
                float value = _hsvValues[2];

                if (saturation > _minimumSaturation && value > .15 ) {
                    _count += 1;

                    if(_count > _minCount) {
                        if (isBlue && (200 <= hue && 240 >= hue)) {
                            keepGoing = false;
                        } else if (!isBlue && (0 <= hue && 40 >= hue)) {
                            keepGoing = false;
                        }
                    }
                } else {
                    keepGoing = true;
                    _count = 0;
                }
            } catch (Exception e) {
                RobotLog.ee("SpinnerStopper::keepGoing()::ColorSensor", e, e.getMessage());
            }


        }

        return keepGoing;

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
