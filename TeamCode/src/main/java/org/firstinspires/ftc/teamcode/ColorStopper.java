package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/2/2017.
 */

class ColorStopper extends DefaultDriver {
    NormalizedColorSensor _sensorRGB;

    float _ambientRed;
    float _ambientGreen;
    float _ambientBlue;

    int _threshold;

    public ColorStopper(NormalizedColorSensor sensorRGB, int threshold, DriverIF driver) {
        super(driver);

        _sensorRGB = sensorRGB;
        _threshold = threshold;
    }

    public void start() {
        super.start();

        NormalizedRGBA colors = _sensorRGB.getNormalizedColors();

        _ambientRed = colors.red;
        _ambientGreen = colors.green;
        _ambientBlue = colors.blue;

        RobotLog.d("ColorStopper::start::ambient: " + _ambientRed + " " + _ambientGreen + " " + _ambientBlue);
    }

    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            NormalizedRGBA colors = _sensorRGB.getNormalizedColors();

            float red = colors.red;
            float green = colors.green;
            float blue = colors.blue;

            RobotLog.d("ColorStopper::keepGoing::color: " + red + " " + green + " " + blue);

            float deltaRed = red - _ambientRed;
            float deltaGreen = green - _ambientGreen;
            float deltaBlue = blue - _ambientBlue;

            RobotLog.d("ColorStopper::keepGoing::delta: " + deltaRed + " " + deltaGreen + " " + deltaBlue);

            keepGoing = Math.max(deltaRed, deltaBlue) < _threshold;
        }

        return keepGoing;
    }

    public float getAmbientRed() {
        return _ambientRed;
    }

    public float getAmbientGreen() {
        return _ambientGreen;
    }

    public float getAmbientBlue() {
        return _ambientBlue;
    }

}
