package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@Autonomous(name = "TestColor", group = "test")
public class TestColor extends OctobotMain {
    NormalizedColorSensor _sensorRGB;
    protected ModernRoboticsI2cGyro _gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our ColorSensor object.
        _sensorRGB = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

//        loadCalibration();

        waitForStart();

        boolean keepGoing = true;

        float lastRed = 0;
        float lastGreen = 0;
        float lastBlue = 0;

        while (keepGoing) {

            NormalizedRGBA colors = _sensorRGB.getNormalizedColors();

            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            float[] hsvValues = new float[3];

            Color.colorToHSV(colors.toColor(), hsvValues);

            float alpha = colors.alpha;
            float red = colors.red;
            float green = colors.green;
            float blue = colors.blue;

            int heading = _gyro.getHeading();

            if(red != lastRed || blue != lastBlue || green != lastGreen) {
                RobotLog.d("TestColor::Colors: " + System.currentTimeMillis() + " " + alpha + " " + red + " " + green + " " + blue);
                lastRed = red;
                lastBlue = blue;
                lastGreen = green;
            }

            idle();
        }
    }
}
