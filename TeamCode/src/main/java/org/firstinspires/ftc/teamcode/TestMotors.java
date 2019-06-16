package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@Autonomous(name = "TestMotors", group = "Test")
@Disabled
public class TestMotors extends LinearOpMode
{
    protected DcMotor _motorA;
    protected DcMotor _motorB;
    protected DcMotor _motorC;
    protected DcMotor _motorD;

//    protected DcMotor _motorE;
//    protected DcMotor _motorF;
//    protected DcMotor _motorG;
//    protected DcMotor _motorH;

    protected GyroSensor _gyroSensor;

    static final int ENCODER_EPSILON = -5;

    static final int MAX_ENCODER = 1000;

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

        resetAllDriveMotorEncoders();

        _motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

//        _motorA.setTargetPosition(4000);
//        _motorB.setTargetPosition(4000);
//        _motorC.setTargetPosition(4000);
//        _motorD.setTargetPosition(4000);
//
//        _motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        _motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        _motorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        _motorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _motorA.setPower(.5f);
        _motorB.setPower(0f);
        _motorC.setPower(0f);
        _motorD.setPower(0f);
        RobotLog.d("AAA0");

        boolean keepGoing = true;

        while (keepGoing) {
            RobotLog.d("keepGoing: " + keepGoing);

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("MotorPosition " + positionA + " " + positionB + " " + positionC + " " + positionD);

            telemetry.addData("0 MotorAPosistion", positionA);
            telemetry.addData("0 MotorBPosistion", positionB);
            telemetry.addData("0 MotorCPosistion", positionC);
            telemetry.addData("0 MotorDPosistion", positionD);
            telemetry.update();
            RobotLog.d("AAA1");
            idle();
            RobotLog.d("AAA2");
            if (positionA > MAX_ENCODER) {
                RobotLog.d("AAA3");
                keepGoing = false;
            }
        }

        RobotLog.d("BAA " + keepGoing);

//        resetAllDriveMotorEncoders();

        _motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();

        RobotLog.d("2 MotorAPosition " + _motorA.getCurrentPosition() + " " + _motorB.getCurrentPosition() + " " + _motorC.getCurrentPosition() + " " + _motorD.getCurrentPosition());

        RobotLog.d("FAA");

        _motorA.setPower(0f);
        _motorB.setPower(0.5f);
        _motorC.setPower(0f);
        _motorD.setPower(0f);

        keepGoing = true;

        while (keepGoing) {
            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("3 MotorAPosition " + positionA + " " + positionB + " " + positionC + " " + positionD);

            telemetry.addData("MotorAPosistion", positionA);
            telemetry.addData("MotorBPosistion", positionB);
            telemetry.addData("MotorCPosistion", positionC);
            telemetry.addData("MotorDPosistion", positionD);
            telemetry.update();

            idle();
            if (positionB > MAX_ENCODER ) {
                keepGoing = false;
            }
        }

//        resetAllDriveMotorEncoders();

        _motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();

        RobotLog.d("2 MotorAPosition " + _motorA.getCurrentPosition() + " " + _motorB.getCurrentPosition() + " " + _motorC.getCurrentPosition() + " " + _motorD.getCurrentPosition());

        RobotLog.d("FAA");

        _motorA.setPower(0f);
        _motorB.setPower(0f);
        _motorC.setPower(0.5f);
        _motorD.setPower(0f);

        keepGoing = true;

        while (keepGoing) {
            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("3 MotorAPosition " + positionA + " " + positionB + " " + positionC + " " + positionD);

            telemetry.addData("MotorAPosistion", positionA);
            telemetry.addData("MotorBPosistion", positionB);
            telemetry.addData("MotorCPosistion", positionC);
            telemetry.addData("MotorDPosistion", positionD);
            telemetry.update();

            idle();
            if (positionC > MAX_ENCODER ) {
                keepGoing = false;
            }
        }

//        resetAllDriveMotorEncoders();

        _motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();

        RobotLog.d("2 MotorAPosition " + _motorA.getCurrentPosition() + " " + _motorB.getCurrentPosition() + " " + _motorC.getCurrentPosition() + " " + _motorD.getCurrentPosition());

        RobotLog.d("FAA");

        _motorA.setPower(0f);
        _motorB.setPower(0f);
        _motorC.setPower(0f);
        _motorD.setPower(0.5f);

        keepGoing = true;

        while (keepGoing) {
            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("3 MotorAPosition " + positionA + " " + positionB + " " + positionC + " " + positionD);

            telemetry.addData("MotorAPosistion", positionA);
            telemetry.addData("MotorBPosistion", positionB);
            telemetry.addData("MotorCPosistion", positionC);
            telemetry.addData("MotorDPosistion", positionD);
            telemetry.update();

            idle();
            if (positionD > MAX_ENCODER ) {
                keepGoing = false;
            }
        }



    }




    public void initializeMotors() throws InterruptedException {

        resetAllDriveMotorEncoders();
    }

    public void initializeSensors() throws InterruptedException {
        _gyroSensor = hardwareMap.gyroSensor.get("gyro");
        _gyroSensor.calibrate();

        while (_gyroSensor.isCalibrating() || _gyroSensor.getHeading() != 0)  {
            idle();
            RobotLog.d("init heading: " + _gyroSensor.getHeading());
        }
    }

    public void resetAllDriveMotorEncoders() throws InterruptedException {
        resetMotorEncoder(_motorA);
        resetMotorEncoder(_motorB);
        resetMotorEncoder(_motorC);
        resetMotorEncoder(_motorD);
    }

    public void resetMotorEncoder(DcMotor dcMotor) throws InterruptedException {
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(dcMotor.getCurrentPosition() != 0) {
            RobotLog.d("resetMotorEncoder::dcMotor.getCurrentPosition(): " + dcMotor.getCurrentPosition());
            idle();
        }
    }

    public void stopMotors() {
        _motorA.setPower(0);
        _motorB.setPower(0);
        _motorC.setPower(0);
        _motorD.setPower(0);
    }

    public int convertInches(float inches) {
        return (int)(2284 * inches / 22.0);
    }

    public static int floorDiv(int x, int y) {
        int r = x / y;

        // if the signs are different and modulo not zero, round down

        if ((x ^ y) < 0 && (r * y != x)) {
            r--;
        }

        return r;
    }

    public static int floorMod(int x, int y) {
        int r = x - floorDiv(x, y) * y;
        return r;
    }

    public void moveForward(int distance, float power) throws InterruptedException {
        RobotLog.d("moveForward::begin");

//        resetAllDriveMotorEncoders();

        idle();

//        _motorE.setTargetPosition(distance);
//        _motorF.setTargetPosition(distance);
//        _motorG.setTargetPosition(distance);
//        _motorH.setTargetPosition(distance);
//
//        waitOneFullHardwareCycle();
//
//        _motorE.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        _motorF.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        _motorG.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        _motorH.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//
//        waitOneFullHardwareCycle();

//        _motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        _motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        _motorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        _motorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        idle();

        _motorA.setPower(power);
        _motorB.setPower(power);
        _motorC.setPower(power);
        _motorD.setPower(power);

        int minDelta;

        idle();

//        int initialHeading = _gyroSensor.getHeading();

        float leftPowerOld = Float.MAX_VALUE;
        float rightPowerOld = Float.MAX_VALUE;

        do {
            idle();

//            int currentHeading = _gyroSensor.getHeading();
//
//            int relativeHeading = currentHeading - initialHeading;
//            relativeHeading = floorMod(relativeHeading + 180, 360) - 180;

            float rightPower = power;
            float leftPower = power;

//            if(relativeHeading != 0) {
//                if(relativeHeading < 0) {
//                    leftPower = power * .8f;
//                    rightPower = power * 1.2f;
//                }
//                else {
//                    leftPower = power * 1.2f;
//                    rightPower = power * .8f;
//                }
//            }
//            else {
//                rightPower = power;
//                leftPower = power;
//            }
//
//            rightPower = Math.min(1, rightPower);
//            leftPower = Math.min(1, leftPower);
//
//            if(rightPower != rightPowerOld || leftPower != leftPowerOld) {
//                _motorA.setPower(rightPower);
//                _motorB.setPower(rightPower);
//                _motorC.setPower(leftPower);
//                _motorD.setPower(leftPower);
//
//                leftPowerOld = leftPower;
//                rightPowerOld = rightPower;
//            }
//
//            RobotLog.d("initialHeading: " + initialHeading  + " currentHeading: " + currentHeading + " relativeHeading: " + relativeHeading + " leftPower: " + leftPower + " rightPower: " + rightPower);

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("positions: " + positionA + " " + positionB + " " + positionC + " " + positionD);

            minDelta = Integer.MAX_VALUE;
            minDelta = Math.min(positionA - distance, minDelta);
            minDelta = Math.min(positionB - distance, minDelta);
            minDelta = Math.min(positionC - distance, minDelta);
            minDelta = Math.min(positionD - distance, minDelta);

            RobotLog.d("min encoder delta: " + minDelta);

        } while(minDelta < ENCODER_EPSILON);

        idle();

        stopMotors();

        idle();

        RobotLog.d("moveForward::end");
    }

    public void moveForwardWithGyro(int distance, float power) throws InterruptedException {
        RobotLog.d("moveForward::begin");

        resetAllDriveMotorEncoders();

        idle();

//        _motorE.setTargetPosition(distance);
//        _motorF.setTargetPosition(distance);
//        _motorG.setTargetPosition(distance);
//        _motorH.setTargetPosition(distance);
//
//        waitOneFullHardwareCycle();
//
//        _motorE.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        _motorF.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        _motorG.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        _motorH.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//
//        waitOneFullHardwareCycle();

        _motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();

//        _motorE.setPower(power);
//        _motorF.setPower(power);
//        _motorG.setPower(power);
//        _motorH.setPower(power);

        int minDelta;

        idle();

        int initialHeading = _gyroSensor.getHeading();

        float leftPowerOld = Float.MAX_VALUE;
        float rightPowerOld = Float.MAX_VALUE;

        do {
            idle();

            int currentHeading = _gyroSensor.getHeading();

            int relativeHeading = currentHeading - initialHeading;
            relativeHeading = floorMod(relativeHeading + 180, 360) - 180;

            float rightPower = power;
            float leftPower = power;

            if(relativeHeading != 0) {
                if(relativeHeading < 0) {
                    leftPower = power * .8f;
                    rightPower = power * 1.2f;
                }
                else {
                    leftPower = power * 1.2f;
                    rightPower = power * .8f;
                }
            }
            else {
                rightPower = power;
                leftPower = power;
            }

            rightPower = Math.min(1, rightPower);
            leftPower = Math.min(1, leftPower);

            if(rightPower != rightPowerOld || leftPower != leftPowerOld) {
                _motorA.setPower(rightPower);
                _motorB.setPower(rightPower);
                _motorC.setPower(leftPower);
                _motorD.setPower(leftPower);

                leftPowerOld = leftPower;
                rightPowerOld = rightPower;
            }

            RobotLog.d("initialHeading: " + initialHeading  + " currentHeading: " + currentHeading + " relativeHeading: " + relativeHeading + " leftPower: " + leftPower + " rightPower: " + rightPower);

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("positions: " + positionA + " " + positionB + " " + positionC + " " + positionD);

            minDelta = Integer.MAX_VALUE;
            minDelta = Math.min(positionA - distance, minDelta);
            minDelta = Math.min(positionB - distance, minDelta);
            minDelta = Math.min(positionC - distance, minDelta);
            minDelta = Math.min(positionD - distance, minDelta);

            RobotLog.d("min encoder delta: " + minDelta);

        } while(minDelta < ENCODER_EPSILON);

        idle();

        stopMotors();

        idle();

        RobotLog.d("moveForward::end");
    }

    public void setRightLeftPower(float rightPower, float leftPower) throws InterruptedException {
        idle();

        _motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();

        RobotLog.d("turn::rightPower: " + rightPower);
        RobotLog.d("turn::leftPower: " + leftPower);

        // write the values to the motors
        _motorA.setPower(rightPower);
        _motorB.setPower(rightPower);
        _motorC.setPower(leftPower);
        _motorD.setPower(leftPower);
    }

    public void turn(int angle, float power) throws InterruptedException {
        RobotLog.d("turn::begin");

        idle();

        int initialHeading = _gyroSensor.getHeading();

        float rightPower;
        float leftPower;

        boolean clockwise = angle > 0;

        if (clockwise) {
            leftPower = -power;
            rightPower = power;
        } else {
            leftPower = power;
            rightPower = -power;
        }

        setRightLeftPower(rightPower, leftPower);

        float headingDelta = angle;

        do {
            idle();

            int currentHeading = _gyroSensor.getHeading();

            int relativeHeading = currentHeading - initialHeading;
            relativeHeading = floorMod(relativeHeading + 180, 360) - 180;

            headingDelta = angle - relativeHeading;

            float powerScale = .25f + .75f * Math.abs(headingDelta / angle );

            RobotLog.d("turn::headingDelta: " + headingDelta + " currentHeading: " + currentHeading + " relativeHeading: " + relativeHeading+ " initialHeading: " + initialHeading);

            if(headingDelta < 0 ^ !clockwise) {
                RobotLog.d("turn::reverse");

                clockwise = !clockwise;

                if (clockwise) {
                    leftPower = - power;
                    rightPower = power;
                } else {
                    leftPower = power;
                    rightPower = -power;
                }
            }

            setRightLeftPower(rightPower * powerScale, leftPower * powerScale);
        }
        while(headingDelta != 0) ;

        idle();

        int finalHeading = _gyroSensor.getHeading();

        RobotLog.d("turn::finalHeading: " + finalHeading);

        idle();

        stopMotors();

        idle();

        RobotLog.d("turn::end");
    }

//    public void startScooper() throws InterruptedException {
//        _motorC.setPower(-1);
//    }
//
//    public void stopScooper() throws InterruptedException {
//        _motorC.setPower(0);
//
//    }

    public void scooper(float power) throws InterruptedException {
        _motorC.setPower(power);
    }

    public void scooperDistance(int distance, float power) throws InterruptedException {
        RobotLog.d("raisearms::begin");

        idle();

        int currentPosition = _motorC.getCurrentPosition();

        RobotLog.d("raisearms::currentPosition " + currentPosition);

//        _motorD.setTargetPosition(distance + currentPosition);

//        waitOneFullHardwareCycle();

//        _motorD.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        idle();


        _motorC.setPower(power);

        int sign = power >0 ? 1 : -1;

        int delta;

        do {
            idle();

            int relativePosition = _motorC.getCurrentPosition() - currentPosition;

            delta = sign * (relativePosition - distance);

            RobotLog.d("raisearms::relativePosition " + relativePosition + " delta " + delta);

        } while (delta <-ENCODER_EPSILON);

        idle();

        _motorC.setPower(0);

        idle();

        RobotLog.d("moveForward::end");
    }

    public void raisearms(int distance, float power) throws InterruptedException {
        RobotLog.d("raisearms::begin");

        idle();

        int currentPosition = _motorD.getCurrentPosition();

        RobotLog.d("raisearms::currentPosition " + currentPosition);

//        _motorD.setTargetPosition(distance + currentPosition);

//        waitOneFullHardwareCycle();

//        _motorD.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        idle();

        _motorD.setPower(power);

        int sign = power >0 ? 1 : -1;

        int delta;

        do {
            idle();

            int relativePosition = _motorD.getCurrentPosition() - currentPosition;

            delta = sign * (relativePosition - distance);

            RobotLog.d("raisearms::relativePosition " + relativePosition + " delta " + delta);

        } while (delta <-ENCODER_EPSILON);

        idle();

        _motorD.setPower(0);

        idle();

        RobotLog.d("moveForward::end");
    }


    public void extendarms(int distance, float power) throws InterruptedException {
        RobotLog.d("extendarms::begin");

        int currentPositionA = _motorA.getCurrentPosition();
        int currentPositionB = _motorB.getCurrentPosition();
//
//        waitOneFullHardwareCycle();

//        _motorA.setTargetPosition(distance);
//        _motorB.setTargetPosition(distance);
//
//        waitOneFullHardwareCycle();
//
//        _motorA.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        _motorB.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        idle();

        _motorA.setPower(power);
        _motorB.setPower(power);

        int minDelta;

        int sign = power >0 ? 1 : -1;

        do {
            idle();

            int relativePositionA = _motorA.getCurrentPosition() - currentPositionA;
            int relativePositionB = _motorB.getCurrentPosition() - currentPositionB;

            int deltaA = sign * (relativePositionA - distance);
            int deltaB = sign * (relativePositionB - distance);

            RobotLog.d("raisearms::relativePositionA " + relativePositionA+ " deltaA " + deltaA);
            RobotLog.d("raisearms::relativePositionB " + relativePositionB+ " deltaB " + deltaB);

            minDelta = Integer.MAX_VALUE;
            minDelta = Math.min(deltaA, minDelta);
//            waitOneFullHardwareCycle();
            minDelta = Math.min(deltaB, minDelta);
//            waitOneFullHardwareCycle();

            RobotLog.d("raisearms::minDelta " + minDelta);

        } while (minDelta < ENCODER_EPSILON);

        idle();

        _motorA.setPower(0);
        _motorB.setPower(0);

        idle();

        RobotLog.d("moveForward::end");
    }

}
