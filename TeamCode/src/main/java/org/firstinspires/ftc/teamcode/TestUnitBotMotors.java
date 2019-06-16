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
@Disabled
@Autonomous(name = "TestUnitBotMotors", group = "MaximumOverdrive")
public class TestUnitBotMotors extends UnitBotMain
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        waitForStart();

        float power = .6f;

        _frontLeft.setPower(power);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(power);

        RobotLog.d("TestUnitBotMotors::power: " + power);

        boolean keepGoing = true;

        while (opModeIsActive()) {
            int positionA = _frontLeft.getCurrentPosition();
            int positionB = _frontRight.getCurrentPosition();
            int positionC = _backLeft.getCurrentPosition();
            int positionD = _backRight.getCurrentPosition();

            telemetry.addData("TestUnitBotMotors::positionA", positionA);
            telemetry.addData("TestUnitBotMotors::positionB", positionB);
            telemetry.addData("TestUnitBotMotors::positionC", positionC);
            telemetry.addData("TestUnitBotMotors::positionD", positionD);

            telemetry.update();

            idle();
        }

    }

}
