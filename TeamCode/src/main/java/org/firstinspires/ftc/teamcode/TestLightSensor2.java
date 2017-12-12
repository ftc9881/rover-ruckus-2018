

/**
 * Created by ftc on 10/19/2016.
 */
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.RobotLog;

/*
*
* This is an example LinearOpMode that shows how to use
* the AdaFruit RGB Sensor. It assumes that the I2C
* cable for the Sensor is connected to an I2C port on the

3 From Wikipedia (https://en.wikipedia.org/wiki/HSL_and_HSV) downloaded on 9/9/15.
32 | FIRST Tech Challenge AdaFruit RGB Sensor Assembly and User’s Guide
AdaFruit RGB Sensor Guide v 003
* Core Device Interface Module.
*
* It also assuems that the LED pin of the Sensor is connected
* to the digital signal pin of a digital port on the
* Core Device Interface Module.
*
* You can use the digital port to turn the Sensor's onboard
* LED on or off.
*
* The op mode assumes that the Core Device Interface Module
* is configured with a name of "dim" and that the AdaFruit color Sensor
* is configured as an I2C device with a name of "color".
*
* It also assumes that the LED pin of the RGB Sensor
* is connected to the signal pin of digital port #5 (zero indexed)
* of the Core Device Interface Module.
*
* You can use the X button on either gamepad to turn the LED on and off.
*
*/
@TeleOp(name = "Test Light Sensor 2", group = "TestSensor")
@Disabled
public class TestLightSensor2 extends LinearOpMode {
    ColorSensor _sensorRGB;
    public DigitalChannel _rgbLED = null;
    // we assume that the LED pin of the RGB Sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();
        _rgbLED = hardwareMap.digitalChannel.get("rgb_led");
        // get a reference to our ColorSensor object.
        _sensorRGB = hardwareMap.colorSensor.get("color");

        RobotLog.d("I2C address: " + _sensorRGB.getI2cAddress().get7Bit());
//        _sensorRGB.setI2cAddress(new I2cAddr(0x52));

        // bEnabled represents the state of the LED.
        boolean bEnabled = true;
        // turn the LED on in the beginning, just so user will know that the Sensor is active.
        _rgbLED.setMode(DigitalChannel.Mode.OUTPUT);
        // wait one cycle.

        sleep(10);
        // wait for the start button to be pressed.
        waitForStart();
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB Sensor.

        int relativeLayoutID = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

        final View relativeLayout = ((Activity)
                hardwareMap.appContext).findViewById(relativeLayoutID);
        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x || gamepad2.x;
            // check for button state transitions.
            if (bCurrState == true && bCurrState != bPrevState) {
                // button is transitioning to a pressed state.
                // print a debug statement.
                //DbgLog.msg("MY_DEBUG - x button was pressed!");
                // update previous state variable.
                bPrevState = bCurrState;
                // on button press, enable the LED.
                bEnabled = true;
            } else if (bCurrState == false && bCurrState != bPrevState) {
                // button is transitioning to a released state.
                // print a debug statement.
                //DbgLog.msg("MY_DEBUG - x button was released!");
                // update previous state variable.
                bPrevState = bCurrState;
                // on button press, enable the LED.
                bEnabled = false;
                // turn off the LED.
                _rgbLED.setState(bEnabled);
            }
            // convert the RGB values to HSV values.
            Color.RGBToHSV((_sensorRGB.red() * 255) / 800, (_sensorRGB.green() * 255) / 800,
                    (_sensorRGB.blue() * 255) / 800, hsvValues);
            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", _sensorRGB.alpha());
            telemetry.addData("Red ", _sensorRGB.red());
            telemetry.addData("Green", _sensorRGB.green());
            telemetry.addData("Blue ", _sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            // change the background color to match the color detected by the RGB Sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            // wait a hardware cycle before iterating.
            sleep(10);
        }
    }
}
