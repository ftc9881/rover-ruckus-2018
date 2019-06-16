package org.firstinspires.ftc.teamcode;



import android.graphics.Color;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;



import java.util.ArrayList;

import java.util.List;



@TeleOp(name = "NEWTele", group = "MaximumOverdrive")
@Disabled
public class NEWTele extends UnitBotMain {


    @Override

    public void runOpMode() throws InterruptedException {

        /*

            Initialize all sensors, motors, servos, etc.

         */


        initialize();

        waitForStart();

        boolean automaticMode = false;

        boolean lastUp = false;

        while (opModeIsActive()) {
            int lastPos = _motorStronkBoi.getCurrentPosition();

            int lastPos1 = _motorExpand1.getCurrentPosition();
            int lastPos2 = _motorExpand2.getCurrentPosition();

            boolean up = gamepad1.dpad_up || gamepad2.dpad_up;

            if(!lastUp && up) {
                automaticMode = !automaticMode;
            }

            lastUp = up;

            telemetry.addLine("automaticMode: " + String.valueOf(automaticMode));

            if(automaticMode) {
                automatic(lastPos, lastPos1, lastPos2);
            }
            else {
                manual(lastPos, lastPos1, lastPos2);
            }

            telemetry.update();
        }
    }

    public void manual(int lastPos, int lastPos1, int lastPos2){


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

        double frontLeft;

        double frontRight;

        double rearLeft;

        double rearRight;

        double extend1;

        double extend2;


        double motorBLPower;

        double motorFLPower;

        double motorFRPower;

        double motorRRPower;

        double motorExtend1Power;

        double motorExtend2Power;



        float powerAdjust;



        String statusMessage = "OK";



        int positionA = _frontLeft.getCurrentPosition();

        int positionB = _frontRight.getCurrentPosition();

        int positionC = _backLeft.getCurrentPosition();

        int positionD = _backRight.getCurrentPosition();

        int positionExtend1 = _motorExpand1.getCurrentPosition();

        int positionExtend2 = _motorExpand2.getCurrentPosition();


        RobotLog.d("UnitBotTeleOp::runOpMode()::Motor Position " + positionA + " " + positionB + " " + positionC + " " + positionD + " " + positionExtend1 + " " + positionExtend2);

        telemetry.addData("Motor Position", positionA + " " + positionB + " " + positionC + " " + positionD + " " + positionExtend1 + " " + positionExtend2);



            /*

                Button Actions

             */
        if(rightTrigger > .05 || leftTrigger > .05) {
            if (rightTrigger > .05) {
                _motorStronkBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (rightBumper) {
                    _motorStronkBoi.setPower(rightTrigger);
                } else {
                    _motorStronkBoi.setPower(rightTrigger / 1.5);
                }
                lastPos = _motorStronkBoi.getCurrentPosition();

            } else if (leftTrigger > .05) {
                _motorStronkBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (rightBumper) {
                    _motorStronkBoi.setPower(-leftTrigger);
                } else {
                    _motorStronkBoi.setPower(-leftTrigger / 1.5);
                }
                lastPos = _motorStronkBoi.getCurrentPosition();
            }
        }
        else if(Math.abs(leftY2) > .05) {
            if (leftY2 > .05) {
                _motorStronkBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                _motorStronkBoi.setPower(leftY2 / 1.5);
                lastPos = _motorStronkBoi.getCurrentPosition();

            } else if (leftY2 < .05) {
                _motorStronkBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                _motorStronkBoi.setPower(leftY2 / 1.5);
                lastPos = _motorStronkBoi.getCurrentPosition();
            }
        }
        else{
            _motorStronkBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _motorStronkBoi.setTargetPosition(lastPos);
        }


        if(left){
            _motorExpand1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand1.setPower(1);
            _motorExpand2.setPower(-1);
            lastPos1 = _motorExpand1.getCurrentPosition();
            lastPos2 = _motorExpand2.getCurrentPosition();
        }
        else if(right){
            _motorExpand1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand1.setPower(-1);
            _motorExpand2.setPower(1);
            lastPos1 = _motorExpand1.getCurrentPosition();
            lastPos2 = _motorExpand2.getCurrentPosition();
        }
        else if(rightY2 < -.1){
            _motorExpand1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand1.setPower(-1);
            _motorExpand2.setPower(1);
            lastPos1 = _motorExpand1.getCurrentPosition();
            lastPos2 = _motorExpand2.getCurrentPosition();
        }
        else if(rightY2 > .1){
            _motorExpand1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand1.setPower(1);
            _motorExpand2.setPower(-1);
            lastPos1 = _motorExpand1.getCurrentPosition();
            lastPos2 = _motorExpand2.getCurrentPosition();
        }else{
            _motorExpand1.setPower(0);
            _motorExpand2.setPower(0);

        }

        if (right2) {
            extendServo(_servoReach1, _servoReach2, _servoGrab1, _servoGrab2, _button1, _button2);
        }else if(left2){
            retractServo(_servoReach1, _servoReach2, _servoGrab1, _servoGrab2);
        }


        if(buttonA){
            _motorIntake.setPower(1);
        } else if(buttonX){
            _motorIntake.setPower(-1);
        }else if(rightBumper2){
            _motorIntake.setPower(-1);
        }else if(leftBumper2){
            _motorIntake.setPower(1);
        }
        else{
            _motorIntake.setPower(0);
        }

        telemetry.addData("Stronk Motor Encoder: ",_motorStronkBoi.getCurrentPosition());

        leftX = -1 * gamepad1.left_stick_x;

        leftY = gamepad1.left_stick_y;


        rightX = -1 * gamepad1.right_stick_x;

        rightY = gamepad1.right_stick_y;


        rightTrigger = gamepad1.right_trigger;

        leftTrigger = gamepad1.left_trigger;


        float Yf = (leftY + rightY) / 2f;

        float Yt = (leftY - rightY) / 2f;

        float strafeX = -(leftX + rightX) / 2f;


        float Kf = 1f;

        float Kt = 1f;

        float Ks = 1f;


        frontLeft = Kf * Yf + Kt * Yt + Ks * strafeX;

        frontRight = Kf * Yf - Kt * Yt - Ks * strafeX;

        rearLeft = Kf * Yf + Kt * Yt - Ks * strafeX;

        rearRight = Kf * Yf - Kt * Yt + Ks * strafeX;


        motorBLPower = RobotControl.convertStickToPower(frontRight);

        motorFLPower = RobotControl.convertStickToPower(rearRight);

        motorFRPower = RobotControl.convertStickToPower(frontLeft);

        motorRRPower = RobotControl.convertStickToPower(rearLeft);


        RobotLog.d("OctobotTeleOp::runOpMode()::Motor Power: " + motorFRPower + " " + motorBLPower + " " + motorRRPower + " " + motorFLPower);

        telemetry.addData("Motor Power", motorFRPower + " " + motorBLPower + " " + motorRRPower + " " + motorFLPower);


        double speed;

        speed = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));


        double angle;

        angle = Math.atan2(-leftY, -leftX);


        telemetry.addLine("Angle: " + angle + " " + (180 * angle / Math.PI));


        double changingSpeed;

        changingSpeed = -1 * rightX;


        frontLeft = speed * (Math.sin(angle - (Math.PI / 4))) + changingSpeed;

        frontRight = speed * (Math.cos(angle - (Math.PI / 4))) - changingSpeed;

        rearLeft = speed * (Math.cos(angle - (Math.PI / 4))) + changingSpeed;

        rearRight = speed * (Math.sin(angle - (Math.PI / 4))) - changingSpeed;

        double maxPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(rearLeft), Math.abs(rearRight)));


        frontLeft /= maxPower;

        frontRight /= maxPower;

        rearLeft /= maxPower;

        rearRight /= maxPower;

        motorRRPower = RobotControl.convertStickToPower(rearRight);

        motorFRPower = RobotControl.convertStickToPower(frontRight);

        motorFLPower = RobotControl.convertStickToPower(frontLeft);

        motorBLPower = RobotControl.convertStickToPower(rearLeft);


        if (rightBumper) {

            powerAdjust = 1f;

            motorFRPower *= powerAdjust;

            motorBLPower *= powerAdjust;

            motorRRPower *= powerAdjust;

            motorFLPower *= powerAdjust;

        } else if (leftBumper) {

            powerAdjust = .3f;

            motorFRPower *= powerAdjust;

            motorBLPower *= powerAdjust;

            motorRRPower *= powerAdjust;

            motorFLPower *= powerAdjust;

        } else {

            powerAdjust = .75f;

            motorFRPower *= powerAdjust;

            motorBLPower *= powerAdjust;

            motorRRPower *= powerAdjust;

            motorFLPower *= powerAdjust;

        }


        _frontRight.setPower(motorFRPower);

        _backLeft.setPower(motorBLPower);

        _backRight.setPower(motorRRPower);

        _frontLeft.setPower(motorFLPower);

    }

    public void automatic(int lastPos, int lastPos1, int lastPos2){


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

        double frontLeft;

        double frontRight;

        double rearLeft;

        double rearRight;

        double extend1;

        double extend2;


        double motorBLPower;

        double motorFLPower;

        double motorFRPower;

        double motorRRPower;

        double motorExtend1Power;

        double motorExtend2Power;

        float powerAdjust;

        if(buttonB){
            transition(_motorStronkBoi, _motorExpand1, _motorExpand2, _servoGrab1, _servoGrab2, _servoReach1, _servoReach2);
        }
        else if(buttonA){
            intake(_motorStronkBoi, _motorExpand1, _motorExpand2, _servoGrab1, _servoGrab2, _servoReach1, _servoReach2);
        }
        else if(buttonX){
            deposit(_motorStronkBoi, _motorExpand1, _motorExpand2, _servoGrab1, _servoGrab2,  _servoReach1, _servoReach2, _button1, _button2);
        }
        else if(buttonY){
            attacc( _motorStronkBoi, _motorExpand1, _motorExpand2, _servoGrab1,  _servoGrab2, _servoReach1, _servoReach2);
        }
        else if(right){
            hang(_motorStronkBoi, _motorExpand1, _motorExpand2, _servoGrab1, _servoGrab2, _servoReach1, _servoReach2, _servoHook);
        }
        else if(left){
            end(_motorStronkBoi, _motorExpand1, _motorExpand2, _servoGrab1, _servoGrab2, _servoReach1, _servoReach2, _servoHook);
        }

        if(rightY2 < -.1){
            _motorExpand1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand1.setPower(-1);
            _motorExpand2.setPower(1);
            lastPos1 = _motorExpand1.getCurrentPosition();
            lastPos2 = _motorExpand2.getCurrentPosition();
        }
        else if(rightY2 > .1){
            _motorExpand1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _motorExpand1.setPower(1);
            _motorExpand2.setPower(-1);
            lastPos1 = _motorExpand1.getCurrentPosition();
            lastPos2 = _motorExpand2.getCurrentPosition();
        }else{
            _motorExpand1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _motorExpand2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _motorExpand1.setTargetPosition(lastPos1);
            _motorExpand2.setTargetPosition(lastPos2);

        }

        if(rightBumper2){
            _motorIntake.setPower(-1);
        }else if(leftBumper2){
            _motorIntake.setPower(1);
        }
        else{
            _motorIntake.setPower(0);
        }

        if(right2){
            _servoGrab1.setPosition(1);
            _servoGrab2.setPosition(1);
        }

        leftX = -1 * gamepad1.left_stick_x;

        leftY = gamepad1.left_stick_y;


        rightX = -1 * gamepad1.right_stick_x;

        rightY = gamepad1.right_stick_y;


        rightTrigger = gamepad1.right_trigger;

        leftTrigger = gamepad1.left_trigger;


        float Yf = (leftY + rightY) / 2f;

        float Yt = (leftY - rightY) / 2f;

        float strafeX = -(leftX + rightX) / 2f;


        float Kf = 1f;

        float Kt = 1f;

        float Ks = 1f;


        frontLeft = Kf * Yf + Kt * Yt + Ks * strafeX;

        frontRight = Kf * Yf - Kt * Yt - Ks * strafeX;

        rearLeft = Kf * Yf + Kt * Yt - Ks * strafeX;

        rearRight = Kf * Yf - Kt * Yt + Ks * strafeX;


        motorBLPower = RobotControl.convertStickToPower(frontRight);

        motorFLPower = RobotControl.convertStickToPower(rearRight);

        motorFRPower = RobotControl.convertStickToPower(frontLeft);

        motorRRPower = RobotControl.convertStickToPower(rearLeft);


        RobotLog.d("OctobotTeleOp::runOpMode()::Motor Power: " + motorFRPower + " " + motorBLPower + " " + motorRRPower + " " + motorFLPower);

        telemetry.addData("Motor Power", motorFRPower + " " + motorBLPower + " " + motorRRPower + " " + motorFLPower);


        double speed;

        speed = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));


        double angle;

        angle = Math.atan2(-leftY, -leftX);


        telemetry.addLine("Angle: " + angle + " " + (180 * angle / Math.PI));


        double changingSpeed;

        changingSpeed = -1 * rightX;


        frontLeft = speed * (Math.sin(angle - (Math.PI / 4))) + changingSpeed;

        frontRight = speed * (Math.cos(angle - (Math.PI / 4))) - changingSpeed;

        rearLeft = speed * (Math.cos(angle - (Math.PI / 4))) + changingSpeed;

        rearRight = speed * (Math.sin(angle - (Math.PI / 4))) - changingSpeed;

        double maxPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(rearLeft), Math.abs(rearRight)));


        frontLeft /= maxPower;

        frontRight /= maxPower;

        rearLeft /= maxPower;

        rearRight /= maxPower;

        motorRRPower = RobotControl.convertStickToPower(rearRight);

        motorFRPower = RobotControl.convertStickToPower(frontRight);

        motorFLPower = RobotControl.convertStickToPower(frontLeft);

        motorBLPower = RobotControl.convertStickToPower(rearLeft);


        if (rightBumper) {

            powerAdjust = 1f;

            motorFRPower *= powerAdjust;

            motorBLPower *= powerAdjust;

            motorRRPower *= powerAdjust;

            motorFLPower *= powerAdjust;

        } else if (leftBumper) {

            powerAdjust = .3f;

            motorFRPower *= powerAdjust;

            motorBLPower *= powerAdjust;

            motorRRPower *= powerAdjust;

            motorFLPower *= powerAdjust;

        } else {

            powerAdjust = .75f;

            motorFRPower *= powerAdjust;

            motorBLPower *= powerAdjust;

            motorRRPower *= powerAdjust;

            motorFLPower *= powerAdjust;

        }


        _frontRight.setPower(motorFRPower);

        _backLeft.setPower(motorBLPower);

        _backRight.setPower(motorRRPower);

        _frontLeft.setPower(motorFLPower);
    }
}
