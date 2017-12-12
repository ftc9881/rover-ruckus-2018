package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.security.KeyStore;

@TeleOp(name = "OctobotTeleOp", group = "octobot")
public class OctobotTeleOp extends OctobotMain {
    static final double GRIPPY_POWER = 1;

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

        _servoLock.setPosition(.5);

        boolean rightServoGrip = true;
        boolean reverseMode = false;
        boolean isRed = true;

        float temp;

        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        float relicGrabberPosition = 0;

        while (opModeIsActive()) {

            /*
                Button Assignments
            */

            boolean rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;
            boolean leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;

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

            String statusMessage = "OK";

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("Motor Position " + positionA + " " + positionB + " " + positionC + " " + positionD);
            telemetry.addData("Motor Posistion", positionA + " " + positionB + " " + positionC + " " + positionD);

            /*
                Button Actions
             */

            if (right && !_button1.getState()) {
                _motorSlide.setPower(-1);
            } else if (left && !_button0.getState()) {
                _motorSlide.setPower(1);
            } else {
                _motorSlide.setPower(0);
            }

            if (right2) {
                _servoRelic.setPosition(.4f);
            }
            else if (left2) {
                _servoRelic.setPosition(.55f);
            }

            if (up) {
                _motorLift.setPower(1);
            } else if (down && !_button3.getState()) {
                _motorLift.setPower(-.5);
            } else{
                _motorLift.setPower(0);
            }

            try {
                NormalizedRGBA colors = _sensorRGB.getNormalizedColors();

                Color.colorToHSV(colors.toColor(), hsvValues);

                float hue = hsvValues[0];
                float saturation = hsvValues[1];

                if (saturation > .1) {
                    if (hue >= 0 && hue <= 90) {
                        isRed = true;
                    } else if (hue <= 260 && hue >= 120) {
                        isRed = false;
                    }
                }
            }
            catch(Exception e) {
                statusMessage = "Color Sensor Problem";
                isRed = false;
            }

            if (buttonY) {
                if(isRed) {
                    _servoTop.setPosition(0);
                }
                else {
                    _servoBottom.setPosition(0);
                }
            } else if (buttonB) {
                if(isRed) {
                    _servoTop.setPosition(1);
                }
                else {
                    _servoBottom.setPosition(1);
                }
            }

            if (buttonX) {
                if(isRed) {
                    _servoBottom.setPosition(0);
                }
                else {
                    _servoTop.setPosition(0);
                }
            } else if (buttonA) {
                if(isRed) {
                    _servoBottom.setPosition(1);
                }
                else {
                    _servoTop.setPosition(1);
                }
            }

            if (rightBumper) {
                _motorSpinner.setPower(.3);
            } else if (leftBumper) {
                _motorSpinner.setPower(-.3);
            } else {
                _motorSpinner.setPower(0);
            }

//            while(rightTrigger>.3){
//                balance();
//            }

            if(rightTrigger2>.5){
                _servoLock.setPosition(1);
            }
            else if(leftTrigger2>.5){
                _servoLock.setPosition(.5);
            }

            if (rightY2>.9) {
                _motorScissor.setPower(-1);
            } else if (rightY2<-.9) {
                _motorScissor.setPower(1);
            } else {
                _motorScissor.setPower(0);
            }

            /*
                Driving
             */

            reverseMode = true;

            if (reverseMode) {
                temp = -1 * leftX;
                leftX = -1 * rightX;
                rightX = temp;

                temp = -1 * leftY;
                leftY = -1 * rightY;
                rightY = temp;
            }

            float Yf = (leftY + rightY) / 2f;
            float Yt = (leftY - rightY) / 2f;
            float strafeX = -(leftX + rightX) / 2f;

            float Kf = 1f;
            float Kt = 1f;
            float Ks = 1f;

            float frontLeft = Kf * Yf + Kt * Yt + Ks * strafeX;
            float frontRight = Kf * Yf - Kt * Yt - Ks * strafeX;
            float rearLeft = Kf * Yf + Kt * Yt - Ks * strafeX;
            float rearRight = Kf * Yf - Kt * Yt + Ks * strafeX;

            float maxPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(rearLeft), Math.abs(rearRight)));

            if (maxPower > 1) {
                frontLeft /= maxPower;
                frontRight /= maxPower;
                rearLeft /= maxPower;
                rearRight /= maxPower;
            }

            float motorAPower = RobotControl.convertStickToPower(frontRight);
            float motorBPower = RobotControl.convertStickToPower(rearRight);
            float motorCPower = RobotControl.convertStickToPower(frontLeft);
            float motorDPower = RobotControl.convertStickToPower(rearLeft);

            float powerAdjust;

            if(leftTrigger>.3){
                powerAdjust = .25f;
            }
            else {
                powerAdjust = 1f;
            }


            motorAPower *= powerAdjust;
            motorBPower *= powerAdjust;
            motorCPower *= powerAdjust;
            motorDPower *= powerAdjust;

            RobotLog.d("Motor Power " + motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);
            telemetry.addData("Motor Power", motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);

            _motorA.setPower(motorAPower);
            _motorB.setPower(motorBPower);
            _motorC.setPower(motorCPower);
            _motorD.setPower(motorDPower);

            NormalizedRGBA colorsArm = _sensorRGBArm.getNormalizedColors();

            Color.colorToHSV(colorsArm.toColor(), hsvValues);

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

            telemetry.addData("Button 0 ", _button0.getState());

            telemetry.addData("Button 1 ", _button1.getState());

            telemetry.addData("Button 2 ", _button2.getState());

            telemetry.addData("Button 3 ", _button3.getState());

            telemetry.addData("isRed", isRed);

            telemetry.addLine("Lift position: " + _motorLift.getCurrentPosition());

            telemetry.addData("Reverse Mode ", reverseMode);

            telemetry.addData("RGB sensor",hsvValues[0] + " " + hsvValues[1] + " " + hsvValues[2]);
            telemetry.update();

            sleep(10);
            idle();
        }

        /*
            Stop Vuforia navigaiton
         */

        stopVuforia();
    }
}