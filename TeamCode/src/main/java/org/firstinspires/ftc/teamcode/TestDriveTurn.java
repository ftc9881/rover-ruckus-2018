package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@Disabled
@Autonomous(name = "TestDriveTurn", group = "MaximumOverdrive")
public class TestDriveTurn extends UnitBotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        waitForStart();

        double initialHeading = getCurrentHeading();

        RobotLog.d("TestDriver::initialHeading::" + initialHeading);

        turn(new IMUTurner(45, 1, _imu1, 2, null, 45, 2, .25),
                true, true);

        RobotLog.d("TestDriver::A");

//        Thread.sleep(250);
//
//        _motorIntake.setPower(1);
    }

}
