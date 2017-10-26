package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@TeleOp(name = "TestDriver", group = "test")
@Disabled
public class TestDriver extends OctobotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        telemetry.addLine("Initializing Vuforia");
        telemetry.update();

        initializeVuforia();

        telemetry.addLine("Calibrating");
        telemetry.update();

//        calibrateSensors();

        telemetry.addLine("Ready");
        telemetry.update();

//        calibrateSensors();

        waitForStart();

        startVuforia();

//        GyroDriver gyroDriver = new GyroDriver(.5, _gyro, .05, RobotControl.convertInches(72), null);
       // LightDriver lightDriver = new LightDriver(.25, _analog0, _led0, _calibrationData._analog0Min , _calibrationData._analog0Max , .5, RobotControl.convertInches(72), null);
        IMUDriver imuDriver = new IMUDriver(.85,0,  _imu, .05, Double.NaN, RobotControl.convertInches(72), null);

        VuforiaTrackableDefaultListener blueNearListener = getBeaconListener(BLUE_NEAR);

        VuforiaBeaconDriver vuforiaBeaconDriver = new VuforiaBeaconDriver(blueNearListener, .5, .05, .1, 5, 300, 20, Integer.MAX_VALUE, null);

        drive(vuforiaBeaconDriver);

        stopVuforia();

//        IMUTurner imuTurnerCW = new IMUTurner(-90, 1, _imu, .1, .1);
//        turn(imuTurnerCW);

//        IMUTurner imuTurnerCCW = new IMUTurner(-90, .25, _imu, 1);
//        turn(imuTurnerCCW);
    }

}
