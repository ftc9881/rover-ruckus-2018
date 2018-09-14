package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "RobotInThreeDays", group = "MaximumOverdrive")
public class RobotInThreeDays extends OctobotMain {


    @Override
    public void runOpMode() throws InterruptedException {
        /*
            Initialize all sensors, motors, servos, etc.
         */

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            /*
                Button Assignments
            */

            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;

            boolean rightBumper2 = gamepad2.right_bumper;
            boolean leftBumper2 = gamepad2.left_bumper;

            boolean right = gamepad1.dpad_right;
            boolean left = gamepad1.dpad_left;

            boolean right2 = gamepad2.dpad_right;
            boolean left2 = gamepad2.dpad_left;

            boolean buttonX = gamepad1.x || gamepad2.x;
            boolean buttonY = gamepad1.y || gamepad2.y;
            boolean buttonA = gamepad1.a || gamepad2.a;
            boolean buttonB = gamepad1.b || gamepad2.b;

            boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean down = gamepad1.dpad_down || gamepad2.dpad_down;

            float rightTrigger = gamepad1.right_trigger;
            float leftTrigger = gamepad1.left_trigger;

            float rightTrigger2 = gamepad2.right_trigger;
            float leftTrigger2 = gamepad2.left_trigger;

            float leftX = gamepad1.left_stick_x;
            float leftY = gamepad1.left_stick_y;

            float rightX = gamepad1.right_stick_x;
            float rightY = gamepad1.right_stick_y;

            float rightY2 = gamepad2.right_stick_y;

            float leftX2 = gamepad2.left_stick_x;
            float leftY2 = gamepad2.left_stick_y;

            String statusMessage = "OK";

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("OctobotTeleOp::runOpMode()::Motor Position " + positionA + " " + positionB + " " + positionC + " " + positionD);
            telemetry.addData("Motor Position", positionA + " " + positionB + " " + positionC + " " + positionD);

            /*
                Button Actions
             */

            if (right2) {

            } else if (left2) {

            }

            if (up) {
                _motorLift.setPower(1);
            } else if (down) {
                _motorLift.setPower(-.5);
            } else {
                _motorLift.setPower(0);
            }

            try {

                if (rightBumper) {

                } else if (leftBumper) {

                }

                if (rightTrigger2 > .5) {

                } else if (leftTrigger2 > .5) {

                }

            /*
                Driving
             */

                boolean reverseMode = true;

                if (reverseMode) {
                    float temp = -1 * leftX;
                    leftX = -1 * rightX;
                    rightX = temp;

                    temp = -1 * leftY;
                    leftY = -1 * rightY;
                    rightY = temp;
                }

                RobotLog.d("OctobotTeleOp::runOpMode()::Stick positions: " + leftY + " " + rightY + " " + leftX + " " + rightX);

                float Yf = (leftY + rightY) / 2f;
                float Yt = (leftY - rightY) / 2f;
                float strafeX = -(leftX + rightX) / 2f;

                float Kf = 1f;
                float Kt = 1f;
                float Ks = 1f;

                double frontLeft = Kf * Yf + Kt * Yt + Ks * strafeX;
                double frontRight = Kf * Yf - Kt * Yt - Ks * strafeX;
                double rearLeft = Kf * Yf + Kt * Yt - Ks * strafeX;
                double rearRight = Kf * Yf - Kt * Yt + Ks * strafeX;

                double maxPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(rearLeft), Math.abs(rearRight)));

                if (maxPower > 1) {
                    frontLeft /= maxPower;
                    frontRight /= maxPower;
                    rearLeft /= maxPower;
                    rearRight /= maxPower;
                }

                double motorAPower = RobotControl.convertStickToPower(frontRight);
                double motorBPower = RobotControl.convertStickToPower(rearRight);
                double motorCPower = RobotControl.convertStickToPower(frontLeft);
                double motorDPower = RobotControl.convertStickToPower(rearLeft);

                float powerAdjust;

                if (leftTrigger > .3) {
                    powerAdjust = .25f;
                } else {
                    powerAdjust = 1f;
                }

                motorAPower *= powerAdjust;
                motorBPower *= powerAdjust;
                motorCPower *= powerAdjust;
                motorDPower *= powerAdjust;

                RobotLog.d("OctobotTeleOp::runOpMode()::Motor Power: " + motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);
                telemetry.addData("Motor Power", motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);

                _motorA.setPower(motorAPower);
                _motorB.setPower(motorBPower);
                _motorC.setPower(motorCPower);
                _motorD.setPower(motorDPower);

                double speed;
                speed = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

                double angle;
                angle = Math.atan2(-leftY, -leftX);

                RobotLog.d("OctobotTeleOp::runOpMode()::Angle: " + angle + " " + (180 * angle / Math.PI));

                double changingSpeed;
                changingSpeed = rightX;

                frontLeft = speed * (Math.sin(angle - (Math.PI / 4))) + changingSpeed;
                frontRight = speed * (Math.cos(angle - (Math.PI / 4))) - changingSpeed;
                rearLeft = speed * (Math.cos(angle - (Math.PI / 4))) + changingSpeed;
                rearRight = speed * (Math.sin(angle - (Math.PI / 4))) - changingSpeed;

                double effectiveSpeed = Math.max(Math.min(1, speed), Math.abs(changingSpeed));

                maxPower = Math.min(effectiveSpeed, Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(rearLeft), Math.abs(rearRight))));

                RobotLog.d("OctobotTeleOp::runOpMode()::maxPower: " + maxPower);

                motorAPower = RobotControl.convertStickToPower(rearLeft / maxPower);
                motorBPower = RobotControl.convertStickToPower(frontLeft / maxPower);
                motorCPower = RobotControl.convertStickToPower(rearRight / maxPower);
                motorDPower = RobotControl.convertStickToPower(frontRight / maxPower);

                if (rightTrigger > .5) {
                    powerAdjust = 1f;
                    motorAPower *= powerAdjust;
                    motorBPower *= powerAdjust;
                    motorCPower *= powerAdjust;
                    motorDPower *= powerAdjust;
                } else if (leftTrigger > .5) {
                    powerAdjust = .3f;
                    motorAPower *= powerAdjust;
                    motorBPower *= powerAdjust;
                    motorCPower *= powerAdjust;
                    motorDPower *= powerAdjust;
                } else {
                    powerAdjust = .75f;
                    motorAPower *= powerAdjust;
                    motorBPower *= powerAdjust;
                    motorCPower *= powerAdjust;
                    motorDPower *= powerAdjust;
                }

                _motorA.setPower(motorAPower);
                _motorB.setPower(motorBPower);
                _motorC.setPower(motorCPower);
                _motorD.setPower(motorDPower);

            /*
                Telemetry
             */

                telemetry.addLine("Status: " + statusMessage);

                if (right || left) {
                    telemetry.addLine("Slide Active");
                } else {
                    telemetry.addLine("Slide Inactive");
                }

                if (up || down) {
                    telemetry.addLine("Lift Active");
                } else {
                    telemetry.addLine("Lift Inactive");
                }

                if (rightBumper || leftBumper) {
                    telemetry.addLine("Spinner Active");
                }
            }finally {
            }
        }
    }
}