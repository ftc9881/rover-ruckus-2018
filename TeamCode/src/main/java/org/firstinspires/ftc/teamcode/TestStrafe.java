package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@TeleOp(name = "TestStrafe", group = "test")
public class TestStrafe extends OctobotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        waitForStart();

        double initialHeading = getCurrentHeading();

        RobotLog.d("TestDriver::initialHeading::" + initialHeading);

        while(opModeIsActive()) {
//            drive(new IMUDriver(0, .4, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(24), null), true, true);
//
//            RobotLog.d("TestDriver::A");
//
//            drive(new IMUDriver(0, -.4, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(24), null), true, true);

            drive(new StrafeDriver(.4f, (int)RobotControl.convertInchesStrafe(24), null), true, true);

            RobotLog.d("TestDriver::A");

            drive(new StrafeDriver(-.4f, (int)RobotControl.convertInchesStrafe(24), null), true, true);
        }
    }

}
