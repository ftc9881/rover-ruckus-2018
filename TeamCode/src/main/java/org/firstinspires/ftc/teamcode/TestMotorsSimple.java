package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@Autonomous(name = "TestMotorsSimple", group = "Test")
public class TestMotorsSimple extends LinearOpMode
{
    protected DcMotor _motorA;
    protected DcMotor _motorB;
    protected DcMotor _motorC;
    protected DcMotor _motorD;

    @Override
    public void runOpMode() throws InterruptedException {
        _motorA = hardwareMap.dcMotor.get("motor_a");
        _motorA.setDirection(DcMotor.Direction.FORWARD);

        _motorB = hardwareMap.dcMotor.get("motor_b");
        _motorB.setDirection(DcMotor.Direction.FORWARD);

        _motorC = hardwareMap.dcMotor.get("motor_c");
        _motorC.setDirection(DcMotor.Direction.FORWARD);

        _motorD = hardwareMap.dcMotor.get("motor_d");
        _motorD.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        _motorA.setPower(1f);
        _motorB.setPower(1f);
        _motorC.setPower(1f);
        _motorD.setPower(1f);

        boolean keepGoing = true;

        while (opModeIsActive()) {
            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            telemetry.addData("0 MotorAPosistion", positionA);
            telemetry.addData("0 MotorBPosistion", positionB);
            telemetry.addData("0 MotorCPosistion", positionC);
            telemetry.addData("0 MotorDPosistion", positionD);
            telemetry.update();

            idle();
        }
    }

}
