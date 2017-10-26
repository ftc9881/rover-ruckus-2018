package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/2/2017.
 */

class ColorStopper extends DefaultDriver {
    TCS34725_ColorSensor _sensorRGB;

    int _ambientRed;
    int _ambientGreen;
    int _ambientBlue;

    int _threshold;

    public ColorStopper(TCS34725_ColorSensor sensorRGB, int threshold, DriverIF driver) {
        super(driver);

        _sensorRGB = sensorRGB;
        _threshold = threshold;
    }

    public void start() {
        super.start();

        _ambientRed = _sensorRGB.redColor();
        _ambientGreen = _sensorRGB.greenColor();
        _ambientBlue = _sensorRGB.blueColor();

        RobotLog.d("ColorStopper::start::ambient: " + _ambientRed + " " + _ambientGreen + " " + _ambientBlue);
    }

    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            int red = _sensorRGB.redColor();
            int green = _sensorRGB.redColor();
            int blue = _sensorRGB.redColor();

            RobotLog.d("ColorStopper::keepGoing::color: " + red + " " + green + " " + blue);

            int deltaRed = red - _ambientRed;
            int deltaGreen = green - _ambientGreen;
            int deltaBlue = blue - _ambientBlue;

            RobotLog.d("ColorStopper::keepGoing::delta: " + deltaRed + " " + deltaGreen + " " + deltaBlue);

            keepGoing = Math.max(deltaRed, deltaBlue) < _threshold;
        }

        return keepGoing;
    }

    public int getAmbientRed() {
        return _ambientRed;
    }

    public int getAmbientGreen() {
        return _ambientGreen;
    }

    public int getAmbientBlue() {
        return _ambientBlue;
    }

}
