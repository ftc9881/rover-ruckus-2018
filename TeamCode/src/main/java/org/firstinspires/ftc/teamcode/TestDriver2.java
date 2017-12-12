package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@TeleOp(name = "TestDriver2", group = "test")
public class TestDriver2 extends OctobotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        waitForStart();

        double initialHeading = getCurrentHeading();

        RobotLog.d("TestDriver2::initialHeading::" + initialHeading);

        turn(new IMUTurner(10, .3, _imu1, .2, .25));

        double newHeading = getCurrentHeading();

        RobotLog.d("TestDriver2::newHeading::" + newHeading);

        drive(new IMUDriver(.85, 0, _imu1, .04, initialHeading, RobotControl.convertInches(36), null));

        RobotLog.d("TestDriver2::A");

        turn(new IMUTurner(10, .3, _imu1, .2, .25));

        double newHeading2 = getCurrentHeading();

        RobotLog.d("TestDriver2::newHeading2::" + newHeading2);

        drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInches(36), null));

        turn(new IMUTurner(-10, .3, _imu1, .2, .25));

        double newHeading3 = getCurrentHeading();

        RobotLog.d("TestDriver2::newHeading3::" + newHeading3);

        drive(new IMUDriver(.85, 0, _imu1, .04, initialHeading, RobotControl.convertInches(36), null));

        RobotLog.d("TestDriver2::A");

        turn(new IMUTurner(-10, .3, _imu1, .2, .25));

        double newHeading4 = getCurrentHeading();

        RobotLog.d("TestDriver2::newHeading4::" + newHeading4);

        drive(new IMUDriver(-.85, 0, _imu1, .04, initialHeading, RobotControl.convertInches(36), null));

        RobotLog.d("TestDriver2::B");
    }

}
