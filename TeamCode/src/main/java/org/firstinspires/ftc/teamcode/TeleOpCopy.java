package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpCopy", group = "octobot")
public class TeleOpCopy extends OctobotMain {
    static final double GRIPPY_POWER = 1;

    static List<Number> GRABBER_POSITIONS = new ArrayList<Number>();

    static {
        GRABBER_POSITIONS.add(0);
        GRABBER_POSITIONS.add(.6);
        GRABBER_POSITIONS.add(1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /*
            Initialize all sensors, motors, servos, etc.
         */

        initialize();

//        resetAllDriveMotorEncoders();
//        resetNonDriveMotorEncoders();
//
//        runUsingEncoders();
//        runNonDriveUsingEncoders();

        /*
            Initialize Vuforia
        */

//        initializeVuforia();

        waitForStart();

        while (opModeIsActive()){
            float joystick1X = gamepad1.left_stick_x;
            float joystick1Y = gamepad1.left_stick_y;
            float joystick2X = gamepad1.right_stick_x;
            float joystick2Y = gamepad1.right_stick_y;

            double power = Math.sqrt((joystick1X * joystick1X) + (joystick1Y * joystick1Y));
            double angle = Math.atan(joystick1Y / joystick1X);

            double changingSpeed = joystick2X;
            double effectiveSpeed = Math.max(Math.min(1, power), Math.abs(changingSpeed));

            double frontLeft = power * (Math.sin(angle - (Math.PI/4))) + changingSpeed;
            double frontRight = power * (Math.cos(angle - (Math.PI/4))) - changingSpeed;
            double rearLeft = power * (Math.cos(angle - (Math.PI/4))) + changingSpeed;
            double rearRight = power * (Math.sin(angle - (Math.PI/4))) - changingSpeed;
            double maxPower = Math.min(effectiveSpeed, Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(rearLeft), Math.abs(rearRight))));

            double motorAPower = RobotControl.convertStickToPower(rearLeft  / maxPower);
            double motorBPower = RobotControl.convertStickToPower(frontLeft / maxPower);
            double motorCPower = RobotControl.convertStickToPower(rearRight / maxPower);
            double motorDPower = RobotControl.convertStickToPower(frontRight / maxPower);

            _motorA.setPower(motorAPower);
            _motorB.setPower(motorBPower);
            _motorC.setPower(motorCPower);
            _motorD.setPower(motorDPower);

            _motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _motorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            _motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            String statusMessage = "OK";

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("OctobotTeleOp::runOpMode()::Motor Position " + positionA + " " + positionB + " " + positionC + " " + positionD);
            telemetry.addData("Motor Position", positionA + " " + positionB + " " + positionC + " " + positionD);

            if (joystick1Y > 0.9){
                _motorA.setPower(1);
                _motorB.setPower(1);
                _motorC.setPower(1);
                _motorD.setPower(1);
            }
            else if (joystick1Y < -0.9){
                _motorA.setPower(-1);
                _motorB.setPower(-1);
                _motorC.setPower(-1);
                _motorD.setPower(-1);
            }
            else if ((joystick1Y <= 0) && (joystick1Y >= -0.9)){
                _motorA.setPower(-power);
                _motorB.setPower(-power);
                _motorC.setPower(-power);
                _motorD.setPower(-power);
            }
            else if ((joystick1Y >= 0) && (joystick1Y <= 0.9)){
                _motorA.setPower(power);
                _motorB.setPower(power);
                _motorC.setPower(power);
                _motorD.setPower(power);
            }
        }
    }
}