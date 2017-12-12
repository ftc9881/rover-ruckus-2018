package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

public class RobotControl {
    static final int ENCODER_EPSILON = -5;

    public static void resetMotorEncoder(DcMotor dcMotor, LinearOpMode opMode) throws InterruptedException {

        int currentPosition = Integer.MAX_VALUE;

        do {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            currentPosition = dcMotor.getCurrentPosition();

            opMode.idle();
        } while(currentPosition != 0);
    }

    public static void resetAllDriveMotorEncoders(DcMotor motorA, DcMotor motorB, DcMotor motorC, DcMotor motorD, LinearOpMode opMode) throws InterruptedException {
        resetMotorEncoder(motorA, opMode);
        resetMotorEncoder(motorB, opMode);
        resetMotorEncoder(motorC, opMode);
        resetMotorEncoder(motorD, opMode);
    }

    public static void runUsingEncoders(DcMotor motorA, DcMotor motorB, DcMotor motorC, DcMotor motorD) throws InterruptedException {
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void stopMotors(DcMotor motorA, DcMotor motorB, DcMotor motorC, DcMotor motorD) {
        motorA.setPower(0);
        motorB.setPower(0);
        motorC.setPower(0);
        motorD.setPower(0);
    }

    public static int convertInches(float inches) {
        return (int)(2284 * inches / 22.0);
    }

    public static int convertInchesStrafe(float inches) {
        return (int)(4400 * inches / 36.0);
    }

    public static float convertStickToPower(float stickValue) {
        return (stickValue > 0 ? 1 : -1) * (float)Math.pow(Math.abs(stickValue), 1.5);
//        return (stickValue > 0 ? 1 : -1) * ((float)Math.pow(Math.abs(stickValue), 10) + (float)(0.4 * Math.abs((stickValue))))/(float)(1.4);
    }

}
