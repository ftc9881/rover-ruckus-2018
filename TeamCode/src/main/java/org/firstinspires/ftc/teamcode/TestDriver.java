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
public class TestDriver extends OctobotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        waitForStart();

        double initialHeading = getCurrentHeading();

        RobotLog.d("TestDriver::initialHeading::" + initialHeading);

        drive(new IMUDriver(0, .85, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(16), null));

        RobotLog.d("TestDriver::A");

        drive(new IMUDriver(0, -.85, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(16), null));

        RobotLog.d("TestDriver::B");

        Thread.sleep(1000);

        turn(new IMUTurner(-90, 1, _imu1, .2, 2));

        RobotLog.d("TestDriver::C");

        turn(new IMUTurner(90, 1, _imu1, .2, 2));

        RobotLog.d("TestDriver::D");

    }

}
