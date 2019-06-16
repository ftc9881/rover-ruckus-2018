package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@Autonomous(name = "TestArmUpDown", group = "MaximumOverdrive")
public class TestArmUpDown extends UnitBotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        waitForStart();

        double initialHeading = getCurrentHeading();

        _motorStronkBoi.setPower(-0.5);
        Thread.sleep(1000);
        _motorStronkBoi.setPower(0.5);
    }

}
