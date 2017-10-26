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
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@Autonomous(name = "TestColor", group = "test")
@Disabled
public class TestColor extends OctobotMain {
    TCS34725_ColorSensor _sensorRGB;
    protected ModernRoboticsI2cGyro _gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our ColorSensor object.
        _sensorRGB = new TCS34725_ColorSensor(hardwareMap, "color");

//        loadCalibration();

        waitForStart();

        boolean keepGoing = true;

        int lastRed = 0;
        int lastGreen = 0;
        int lastBlue = 0;
        int lastHeading = 10000;

        while (keepGoing) {
            int alpha = _sensorRGB.clearColor();
            int red = _sensorRGB.redColor();
            int green = _sensorRGB.greenColor();
            int blue = _sensorRGB.blueColor();

            int heading = _gyro.getHeading();

            if(red != lastRed || blue != lastBlue || green != lastGreen) {
                RobotLog.d("TestColor::Colors: " + System.currentTimeMillis() + " " + alpha + " " + red + " " + green + " " + blue);
                lastRed = red;
                lastBlue = blue;
                lastGreen = green;
//                telemetry.addData("Clear", alpha);
//                telemetry.addData("Red  ", red);
//                telemetry.addData("Green", green);
//                telemetry.addData("Blue ", blue);
//                telemetry.update();
            }

            if(heading != lastHeading) {
                RobotLog.d("TestColor::Heading: " + System.currentTimeMillis() + " " + heading);
                lastHeading = heading;
            }

            idle();
        }
    }
}
