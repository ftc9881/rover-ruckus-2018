package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

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

        waitForStart();

        double initialHeading = getCurrentHeading();

        RobotLog.d("TestDriver::initialHeading::" + initialHeading);

        drive(new IMUDriver(0, .85, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(16), null), true, true);

        RobotLog.d("TestDriver::A");

        drive(new IMUDriver(0, -.85, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(16), null), true, true);

        RobotLog.d("TestDriver::B");

        Thread.sleep(1000);

        turn(new IMUTurner(-90, 1, _imu1, 2, null), true, true);

        RobotLog.d("TestDriver::C");

        turn(new IMUTurner(90, 1, _imu1, 2, null), true, true);

        RobotLog.d("TestDriver::D");

    }

}
