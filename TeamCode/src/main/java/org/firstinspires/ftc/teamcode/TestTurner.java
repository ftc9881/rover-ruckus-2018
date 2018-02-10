package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@TeleOp(name = "TestTurner", group = "test")
public class TestTurner extends OctobotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        double power = .6;
        boolean runUsingEncoders = true;

        RobotLog.d("TestTurner::180");

        double lastHeading = getCurrentHeading();

        turn(new IMUTurner(180, power, _imu1, .25, null), runUsingEncoders, true);

        sleep(1000);

        double currentHeading = getCurrentHeading();
        double headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        RobotLog.d("TestTurner::90");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(90, power, _imu1, .25, null), runUsingEncoders, true);

        sleep(1000);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        RobotLog.d("TestTurner::45");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(45, power, _imu1, .25, null), runUsingEncoders, true);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        sleep(1000);

        RobotLog.d("TestTurner::10");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(10, power, _imu1, .25, null), runUsingEncoders, true);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        sleep(1000);

        RobotLog.d("TestTurner::5");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(5, power, _imu1, .25, null), runUsingEncoders, true);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        sleep(1000);

        RobotLog.d("TestTurner::1");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(1, power, _imu1, .25, null), runUsingEncoders, true);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        turn(new IMUTurner(-180, power, _imu1, .25, null), runUsingEncoders, true);

        sleep(1000);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        RobotLog.d("TestTurner::90");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(-90, power, _imu1, .25, null), runUsingEncoders, true);

        sleep(1000);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        RobotLog.d("TestTurner::45");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(-45, power, _imu1, .25, null), runUsingEncoders, true);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        sleep(1000);

        RobotLog.d("TestTurner::10");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(-10, power, _imu1, .25, null), runUsingEncoders, true);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        sleep(1000);

        RobotLog.d("TestTurner::5");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(-5, power, _imu1, .25, null), runUsingEncoders, true);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

        sleep(1000);

        RobotLog.d("TestTurner::1");

        lastHeading = getCurrentHeading();

        turn(new IMUTurner(-1, power, _imu1, .25, null), runUsingEncoders, true);

        currentHeading = getCurrentHeading();
        headingChange = currentHeading - lastHeading;
        RobotLog.d("TestTurner::headingChange: " + headingChange);

    }

}
