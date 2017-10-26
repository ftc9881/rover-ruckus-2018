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
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(dcMotor.getCurrentPosition() != 0) {
            opMode.idle();
        }
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
    }

    public static void moveWithGyro(int target, float power, DcMotor motorA, DcMotor motorB, DcMotor motorC, DcMotor motorD,
                                    ModernRoboticsI2cGyro gyro, AnalogInput analogInput, double brightness,
                                    DigitalChannel buttonInput, TCS34725_ColorSensor colorSensor, LinearOpMode opMode, boolean strafe ) throws InterruptedException {
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;



        resetAllDriveMotorEncoders(motorA, motorB, motorC, motorD, opMode);
        runUsingEncoders(motorA,motorB, motorC, motorD);

        gyro.resetZAxisIntegrator();

        RobotLog.d("Initial integrated Z Value: " + gyro.getIntegratedZValue());



        if (strafe) {
            motorA.setPower(power);
            motorB.setPower(-power);
            motorC.setPower(-power);
            motorD.setPower(power);
        }
        else {
           motorA.setPower(power);
           motorB.setPower(power);
           motorC.setPower(power);
           motorD.setPower(power);
       }

        boolean keepGoing = true;

        double lightScaleFactor = 1;
        double voltageEWMA = 1;
        boolean refinementMode = false;

        while (keepGoing) {
            if(colorSensor != null) {
                float hsvValues[] = {0F, 0F, 0F};
                Color.RGBToHSV((colorSensor.redColor() * 255) / 800, (colorSensor.greenColor() * 255) / 800, (colorSensor.blueColor() * 255) / 800, hsvValues);

                RobotLog.d("Colors: " + colorSensor.clearColor() + " " + colorSensor.redColor() + " " + colorSensor.greenColor() + " " + colorSensor.blueColor() + " " + hsvValues[0] + " " +  hsvValues[1] + " " + hsvValues[2]);
            }


            heading = gyro.getHeading();
            angleZ = gyro.getIntegratedZValue();

//            opMode.telemetry.addData("0 MotorAPosistion", positionA);
//            opMode.telemetry.addData("0 MotorBPosistion", positionB);
//            opMode.telemetry.addData("0 MotorCPosistion", positionC);
//            opMode.telemetry.addData("0 MotorDPosistion", positionD);
//            opMode.telemetry.addData("Relative Heading:", angleZ);
//            if (analogInput != null) {
//                opMode.telemetry.addData("Brightness;", analogInput.getVoltage());
//            }
//            opMode.telemetry.update();

            opMode.idle();

            RobotLog.d("Heading: " + heading);
            RobotLog.d("Integrated Z Value: " + angleZ);

            if (strafe) {
                motorA.setPower(power * lightScaleFactor);
                motorB.setPower(-power * lightScaleFactor);
                motorC.setPower(-power * lightScaleFactor);
                motorD.setPower(power * lightScaleFactor);
            }
            else {
                double gyroScaleFactor = angleZ / 20f;

                RobotLog.d("gyroScaleFactor: " + gyroScaleFactor);

                motorA.setPower(power * ( 1 + gyroScaleFactor) * lightScaleFactor);
                motorB.setPower(power * ( 1 + gyroScaleFactor) * lightScaleFactor);
                motorC.setPower(power * ( 1 - gyroScaleFactor) * lightScaleFactor);
                motorD.setPower(power * ( 1 - gyroScaleFactor) * lightScaleFactor);
            }

            int positionA = motorA.getCurrentPosition();
            int positionB = motorB.getCurrentPosition();
            int positionC = motorC.getCurrentPosition();
            int positionD = motorD.getCurrentPosition();

            RobotLog.d("MotorPosition: " + positionA + " " + positionB + " " + positionC + " " + positionD);

            if ((Math.abs(positionA) + Math.abs(positionB) + Math.abs(positionC) + Math.abs(positionD)) / 4 > target ) {
                keepGoing = false;
            }

            if(analogInput != null) {
                double voltage = analogInput.getVoltage();

                voltageEWMA = voltageEWMA * .9 + voltage * .1;

                RobotLog.d("Voltage: " + voltage + " " + voltageEWMA);

                if (voltage > brightness) { // * .85 && voltage <= brightness * 1.15) {
                    keepGoing = false;
                }
                else if(voltage > brightness) {
//                    refinementMode = true;
                }

                if(refinementMode) {
                    if (voltageEWMA < brightness) {
                        lightScaleFactor = .5; // .1 + .25 * (brightness - voltageEWMA) / brightness;
                    } else {
                        lightScaleFactor = -.5; // (.1 + .25 * (voltageEWMA - brightness) / brightness);
                    }
                }

                RobotLog.d("lightScaleFactor: " + lightScaleFactor);
            }

            if(buttonInput != null) {
                RobotLog.d("buttonInput " + buttonInput.getState());
                if(buttonInput.getState()) {
                    keepGoing = false;
                }
            }

        }

        RobotLog.d("BAA " + keepGoing);

        stopMotors(motorA, motorB, motorC, motorD);
    }

    public static void moveWithGyroTest(int target, float power, DcMotor motorA, DcMotor motorB, DcMotor motorC, DcMotor motorD,
                                    ModernRoboticsI2cGyro gyro, AnalogInput analogInput, double brightness,
                                    DigitalChannel buttonInput, ColorSensor colorSensor, LinearOpMode opMode, boolean strafe ) throws InterruptedException {
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;



        resetAllDriveMotorEncoders(motorA, motorB, motorC, motorD, opMode);
        runUsingEncoders(motorA,motorB, motorC, motorD);

        gyro.resetZAxisIntegrator();

        RobotLog.d("Initial integrated Z Value: " + gyro.getIntegratedZValue());



        if (strafe) {
            motorA.setPower(power);
            motorB.setPower(-power);
            motorC.setPower(-power);
            motorD.setPower(power);
        }
        else {
            motorA.setPower(power);
            motorB.setPower(power);
            motorC.setPower(power);
            motorD.setPower(power);
        }


        boolean keepGoing = true;

        double lightScaleFactor = 1;
        double voltageEWMA = 1;
        boolean refinementMode = false;



        while (keepGoing) {
            if(colorSensor != null) {
                float hsvValues[] = {0F, 0F, 0F};
                Color.RGBToHSV((colorSensor.red() * 255) / 800, (colorSensor.green() * 255) / 800, (colorSensor.blue() * 255) / 800, hsvValues);

                RobotLog.d("Colors: " + colorSensor.alpha() + " " + colorSensor.red() + " " + colorSensor.green() + " " + colorSensor.blue() + " " + hsvValues[0] + " " +  hsvValues[1] + " " + hsvValues[2]);
            }

            heading = gyro.getHeading();
            angleZ = gyro.getIntegratedZValue();

//            opMode.telemetry.addData("0 MotorAPosistion", positionA);
//            opMode.telemetry.addData("0 MotorBPosistion", positionB);
//            opMode.telemetry.addData("0 MotorCPosistion", positionC);
//            opMode.telemetry.addData("0 MotorDPosistion", positionD);
//            opMode.telemetry.addData("Relative Heading:", angleZ);
//            if (analogInput != null) {
//                opMode.telemetry.addData("Brightness;", analogInput.getVoltage());
//            }
//            opMode.telemetry.update();

            opMode.idle();

            RobotLog.d("Heading: " + heading);
            RobotLog.d("Integrated Z Value: " + angleZ);

            if (strafe) {
                motorA.setPower(power * lightScaleFactor);
                motorB.setPower(-power * lightScaleFactor);
                motorC.setPower(-power * lightScaleFactor);
                motorD.setPower(power * lightScaleFactor);
            }
            else {
                double gyroScaleFactor = angleZ / 20f;

                RobotLog.d("gyroScaleFactor: " + gyroScaleFactor);

                motorA.setPower(power * ( 1 + gyroScaleFactor) * lightScaleFactor);
                motorB.setPower(power * ( 1 + gyroScaleFactor) * lightScaleFactor);
                motorC.setPower(power * ( 1 - gyroScaleFactor) * lightScaleFactor);
                motorD.setPower(power * ( 1 - gyroScaleFactor) * lightScaleFactor);
            }

            int positionA = motorA.getCurrentPosition();
            int positionB = motorB.getCurrentPosition();
            int positionC = motorC.getCurrentPosition();
            int positionD = motorD.getCurrentPosition();

            RobotLog.d("MotorPosition: " + positionA + " " + positionB + " " + positionC + " " + positionD);

            if ((Math.abs(positionA) + Math.abs(positionB) + Math.abs(positionC) + Math.abs(positionD)) / 4 > target ) {
                keepGoing = false;
            }

            if(analogInput != null) {
                double voltage = analogInput.getVoltage();

                voltageEWMA = voltageEWMA * .9 + voltage * .1;

                RobotLog.d("Voltage: " + voltage + " " + voltageEWMA);

                if (voltage > brightness) { // * .85 && voltage <= brightness * 1.15) {
                    keepGoing = false;
                }
                else if(voltage > brightness) {
//                    refinementMode = true;
                }

                if(refinementMode) {
                    if (voltageEWMA < brightness) {
                        lightScaleFactor = .5; // .1 + .25 * (brightness - voltageEWMA) / brightness;
                    } else {
                        lightScaleFactor = -.5; // (.1 + .25 * (voltageEWMA - brightness) / brightness);
                    }
                }

                RobotLog.d("lightScaleFactor: " + lightScaleFactor);
            }

            if(buttonInput != null) {
                RobotLog.d("buttonInput " + buttonInput.getState());
                if(buttonInput.getState()) {
                    keepGoing = false;
                }
            }
            if (colorSensor.blue() > 180) {
                keepGoing = false;
            }

        }

        RobotLog.d("BAA " + keepGoing);

        stopMotors(motorA, motorB, motorC, motorD);
    }

    public static void moveWithLight(int target, float power, DcMotor motorA, DcMotor motorB, DcMotor motorC, DcMotor motorD,
                                     AnalogInput analogInput, double brightness, DriverIF stopper,
                                     LinearOpMode opMode, double strength ) throws InterruptedException {
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;

        resetAllDriveMotorEncoders(motorA, motorB, motorC, motorD, opMode);
        runUsingEncoders(motorA,motorB, motorC, motorD);

        boolean keepGoing = true;

        double voltageEWMA = analogInput.getVoltage();

        if(stopper != null) {
            stopper.start();
        }

        while (keepGoing) {
            RobotLog.d("keepGoing: " + keepGoing);

            int positionA = motorA.getCurrentPosition();
            int positionB = motorB.getCurrentPosition();
            int positionC = motorC.getCurrentPosition();
            int positionD = motorD.getCurrentPosition();

            double voltage = analogInput.getVoltage();

            voltageEWMA = voltageEWMA * .8 + voltage * .2;

            double difference = (voltageEWMA - brightness) / brightness;

//            opMode.telemetry.addData("0 MotorAPosistion", positionA);
//            opMode.telemetry.addData("0 MotorBPosistion", positionB);
//            opMode.telemetry.addData("0 MotorCPosistion", positionC);
//            opMode.telemetry.addData("0 MotorDPosistion", positionD);
//            opMode.telemetry.addData("Relative Heading:", angleZ);
//            opMode.telemetry.addData("Brightness;", analogInput.getVoltage());
//            opMode.telemetry.addData("Difference", difference);
//            opMode.telemetry.update();
            opMode.idle();

            RobotLog.d("MotorPosition " + positionA + " " + positionB + " " + positionC + " " + positionD);
            RobotLog.d("Difference: " + difference);
            RobotLog.d("Voltage " + voltage + " " + voltageEWMA + " " + difference);

            motorA.setPower(power - strength * difference);
            motorB.setPower(power - strength * difference);
            motorC.setPower(power + strength * difference);
            motorD.setPower(power + strength * difference);

            int position = (Math.abs(positionA) + Math.abs(positionB) + Math.abs(positionC) + Math.abs(positionD)) / 4;

            if (position > target) {
                keepGoing = false;
            }

            if(stopper != null) {
                if(!stopper.keepGoing(position)) {
                    keepGoing = false;
                }
            }
        }

        RobotLog.d("BAA " + keepGoing);

        stopMotors(motorA, motorB, motorC, motorD);
    }

    public static void turnWithGyro(int degrees, float power,
                                DcMotor motorA, DcMotor motorB, DcMotor motorC, DcMotor motorD,
                                ModernRoboticsI2cGyro gyro, LinearOpMode opMode, AnalogInput analogInput, double brightness) throws InterruptedException {

        resetAllDriveMotorEncoders(motorA, motorB, motorC, motorD, opMode);
        runUsingEncoders(motorA,motorB, motorC, motorD);

        opMode.idle();

        RobotLog.d("Motor Position: " + motorA.getCurrentPosition() + " " + motorB.getCurrentPosition() + " " + motorC.getCurrentPosition() + " " + motorD.getCurrentPosition());

        if(analogInput != null) {
            RobotLog.d("Brightness:" + analogInput.getVoltage());
        }

        int angleZ;

        gyro.resetZAxisIntegrator();

        float currentPower = power;

        if(degrees < 0) {
            power *= -1;
        }

        boolean notDone = true;

        while(notDone) {
            RobotLog.d("Current Power: " + currentPower);

            boolean keepGoing = true;

            while (keepGoing) {
//                int positionA = motorA.getCurrentPosition();
//                int positionB = motorB.getCurrentPosition();
//                int positionC = motorC.getCurrentPosition();
//                int positionD = motorD.getCurrentPosition();

                angleZ = gyro.getIntegratedZValue();

                //            RobotLog.d("3 MotorAPosition " + positionA + " " + positionB + " " + positionC + " " + positionD);

                //            opMode.telemetry.addData("angleZ", angleZ);
                //            opMode.telemetry.addData("MotorAPosistion", positionA);
                //            opMode.telemetry.addData("MotorBPosistion", positionB);
                //            opMode.telemetry.addData("MotorCPosistion", positionC);
                //            opMode.telemetry.addData("MotorDPosistion", positionD);

                RobotLog.d("turnWithGyro::angleZ = " + angleZ + " " + System.currentTimeMillis());

//                opMode.telemetry.update();

                if (analogInput != null && analogInput.getVoltage() > brightness) {
                    keepGoing = false;
                    notDone = false;
                }

                float scaleFactor = .1f + .9f * Math.abs(((float)(degrees - angleZ)) / (float)degrees);

                RobotLog.d("turnWithGyro::scaleFactor = " + scaleFactor);

                motorA.setPower(-currentPower * scaleFactor);
                motorB.setPower(-currentPower * scaleFactor);
                motorC.setPower(currentPower * scaleFactor);
                motorD.setPower(currentPower * scaleFactor);

                boolean clockwise = (currentPower > 0);

                if ((angleZ >= degrees && clockwise) || (angleZ <= degrees && !clockwise)) {
                    keepGoing = false;
                }

                opMode.idle();

                Thread.sleep(20);
            }

            stopMotors(motorA, motorB, motorC, motorD);

            angleZ = gyro.getIntegratedZValue();

            if(angleZ == degrees) {
                notDone = false;
            }
            else {
                RobotLog.d("turnWithGyro::Reverse direction");
                currentPower = -currentPower;
            }

            Thread.sleep(20);
        }
    }

}
