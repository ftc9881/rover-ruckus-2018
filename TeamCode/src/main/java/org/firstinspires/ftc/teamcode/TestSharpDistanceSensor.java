package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "TestSharpDistanceSensor", group = "Test")
public class TestSharpDistanceSensor extends OctobotMain {
//    //lookup table to find distance compared to voltage based on our measurements. We may need to
//    // recalculate these using a more exact measurement tool, but they are within 1 cm.
//    //
//    // We measured these by taking physical measurements with the sensor.
//
//    static double[] distanceCm = {3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30};
//    // These need to be re-calibrated.  Will update in next release.
//    static double[] voltage = {3.051757813, 2.685546875, 2.255859375, 1.93359375, 1.684570313,
//            1.489257813, 1.357421875, 1.240234375, 1.137695313, 1.044921875,
//            0.9814453125, 0.9228515625, 0.8740234375, 0.8349609375, 0.7958984375,
//            0.76171875, 0.7177734375, 0.6982421875, 0.6591796875, 0.6396484375,
//            0.6201171875, 0.576171875, 0.556640625, 0.5322265625, 0.5126953125,
//            0.5126953125, 0.4931640625, 0.4736328125};

    /*
      * Main loop
      */
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the hardware
         */

	    initialize();

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            // Reading voltage
            double voltreadingLeft = (float) _irSensorLeft.getVoltage();
            double voltreadingRight = (float) _irSensorRight.getVoltage();
            //convert voltage to distance (cm)

            double distanceLeft = _irSensorLeft.getDistance();
            double distanceRight = _irSensorRight.getDistance();

            telemetry.addData("time", time);
            telemetry.addData("voltage", "%f %f", voltreadingLeft, voltreadingRight);
            // this is our calculated value
            telemetry.addData("distance", "%f %f", distanceLeft, distanceRight);
            telemetry.update();

            RobotLog.d("SharpDistanceSensor::voltage " + voltreadingLeft + " " + voltreadingRight);
            RobotLog.d("SharpDistanceSensor::distance " + distanceLeft + " " + distanceRight);
        }
    }


}
