package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "SwerveTeleOp", group = "octobot")
public class SwerveTeleOp extends OctobotMain {

    @Override
    public void runOpMode() throws InterruptedException {
        /*
            Initialize all sensors, motors, servos, etc.
         */

        initialize();

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

            _servoA.setPosition(angle/180);
//            _servoB.setPosition(angle/180);
//            _servoC.setPosition(angle/180);
//            _servoD.setPosition(angle/180);

            _motorA.setPower(power);
            _motorB.setPower(power);
            _motorC.setPower(power);
            _motorD.setPower(power);

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
            else if ((joystick1Y >= -0.9) && (joystick1Y < 0)){
                _motorA.setPower(-power);
                _motorB.setPower(-power);
                _motorC.setPower(-power);
                _motorD.setPower(-power);
            }
            else if ((joystick1Y <= 0.9) && (joystick1Y > 0)){
                _motorA.setPower(power);
                _motorB.setPower(power);
                _motorC.setPower(power);
                _motorD.setPower(power);
            }

        }


    }
}