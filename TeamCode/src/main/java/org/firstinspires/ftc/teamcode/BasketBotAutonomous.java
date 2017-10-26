package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
public abstract class BasketBotAutonomous extends LinearOpMode {
    protected DcMotor _motorA;
    protected DcMotor _motorB;
    protected DcMotor _motorC;
    protected DcMotor _motorD;

    protected DcMotor _motorE;
    protected DcMotor _motorF;
    protected DcMotor _motorG;
    protected DcMotor _motorH;

    protected GyroSensor _gyroSensor;

    static final int ENCODER_EPSILON = -5;


    public void initializeMotors() throws InterruptedException {
        _motorA = hardwareMap.dcMotor.get("motor_a");
        _motorA.setDirection(DcMotor.Direction.REVERSE);
        _motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        _motorB = hardwareMap.dcMotor.get("motor_b");
        _motorB.setDirection(DcMotor.Direction.FORWARD);
        _motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        _motorC = hardwareMap.dcMotor.get("motor_c");
        _motorC.setDirection(DcMotor.Direction.FORWARD);
        _motorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        _motorD = hardwareMap.dcMotor.get("motor_d");
        _motorD.setDirection(DcMotor.Direction.FORWARD);
        _motorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        _motorE = hardwareMap.dcMotor.get("motor_e");
        _motorE.setDirection(DcMotor.Direction.REVERSE);

        _motorF = hardwareMap.dcMotor.get("motor_f");
        _motorF.setDirection(DcMotor.Direction.REVERSE);

        _motorG = hardwareMap.dcMotor.get("motor_g");
        _motorG.setDirection(DcMotor.Direction.FORWARD);

        _motorH = hardwareMap.dcMotor.get("motor_h");
        _motorH.setDirection(DcMotor.Direction.FORWARD);

//        RobotLog.d()

        resetAllDriveMotorEncoders();
    }

    public void initializeSensors() throws InterruptedException {
        _gyroSensor = hardwareMap.gyroSensor.get("gyro");
        _gyroSensor.calibrate();

        while (_gyroSensor.isCalibrating() || _gyroSensor.getHeading() != 0)  {
            waitOneFullHardwareCycle();
            RobotLog.d("init heading: " + _gyroSensor.getHeading());
        }
    }

    public void resetAllDriveMotorEncoders() throws InterruptedException {
//        resetMotorEncoder(_motorA);
//        resetMotorEncoder(_motorB);
//        resetMotorEncoder(_motorC);
//        resetMotorEncoder(_motorD);

        resetMotorEncoder(_motorE);
        resetMotorEncoder(_motorF);
        resetMotorEncoder(_motorG);
        resetMotorEncoder(_motorH);
    }

    public void resetMotorEncoder(DcMotor dcMotor) throws InterruptedException {
        dcMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        while(dcMotor.getCurrentPosition() != 0) {
            waitOneFullHardwareCycle();
        }
    }

    public void stopMotors() {
        _motorE.setPower(0);
        _motorF.setPower(0);
        _motorG.setPower(0);
        _motorH.setPower(0);
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

        resetAllDriveMotorEncoders();

        waitOneFullHardwareCycle();

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

        _motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        _motorF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        _motorG.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        _motorH.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        waitOneFullHardwareCycle();

        _motorE.setPower(power);
        _motorF.setPower(power);
        _motorG.setPower(power);
        _motorH.setPower(power);

        int minDelta;

        waitOneFullHardwareCycle();

        int initialHeading = _gyroSensor.getHeading();

        float leftPowerOld = Float.MAX_VALUE;
        float rightPowerOld = Float.MAX_VALUE;

        do {
            waitOneFullHardwareCycle();

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
                _motorE.setPower(rightPower);
                _motorF.setPower(rightPower);
                _motorG.setPower(leftPower);
                _motorH.setPower(leftPower);

                leftPowerOld = leftPower;
                rightPowerOld = rightPower;
            }

            RobotLog.d("initialHeading: " + initialHeading  + " currentHeading: " + currentHeading + " relativeHeading: " + relativeHeading + " leftPower: " + leftPower + " rightPower: " + rightPower);

            int positionE = _motorE.getCurrentPosition();
            int positionF = _motorF.getCurrentPosition();
            int positionG = _motorG.getCurrentPosition();
            int positionH = _motorH.getCurrentPosition();

            RobotLog.d("positions: " + positionE + " " + positionF + " " + positionG + " " + positionH);

            minDelta = Integer.MAX_VALUE;
            minDelta = Math.min(positionE - distance, minDelta);
            minDelta = Math.min(positionF - distance, minDelta);
            minDelta = Math.min(positionG - distance, minDelta);
            minDelta = Math.min(positionH - distance, minDelta);

            RobotLog.d("min encoder delta: " + minDelta);

        } while(minDelta < ENCODER_EPSILON);

        waitOneFullHardwareCycle();

        stopMotors();

        waitOneFullHardwareCycle();

        RobotLog.d("moveForward::end");
    }

    public void setRightLeftPower(float rightPower, float leftPower) throws InterruptedException {
        waitOneFullHardwareCycle();

        _motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        _motorF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        _motorG.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        _motorH.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        waitOneFullHardwareCycle();

        RobotLog.d("turn::rightPower: " + rightPower);
        RobotLog.d("turn::leftPower: " + leftPower);

        // write the values to the motors
        _motorE.setPower(rightPower);
        _motorF.setPower(rightPower);
        _motorG.setPower(leftPower);
        _motorH.setPower(leftPower);
    }

    public void turn(int angle, float power) throws InterruptedException {
        RobotLog.d("turn::begin");

        waitOneFullHardwareCycle();

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
            waitOneFullHardwareCycle();

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

        waitOneFullHardwareCycle();

        int finalHeading = _gyroSensor.getHeading();

        RobotLog.d("turn::finalHeading: " + finalHeading);

        waitOneFullHardwareCycle();

        stopMotors();

        waitOneFullHardwareCycle();

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

        waitOneFullHardwareCycle();

        int currentPosition = _motorC.getCurrentPosition();

        RobotLog.d("raisearms::currentPosition " + currentPosition);

//        _motorD.setTargetPosition(distance + currentPosition);

//        waitOneFullHardwareCycle();

//        _motorD.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        waitOneFullHardwareCycle();

        _motorC.setPower(power);

        int sign = power >0 ? 1 : -1;

        int delta;

        do {
            waitOneFullHardwareCycle();

            int relativePosition = _motorC.getCurrentPosition() - currentPosition;

            delta = sign * (relativePosition - distance);

            RobotLog.d("raisearms::relativePosition " + relativePosition + " delta " + delta);

        } while (delta <-ENCODER_EPSILON);

        waitOneFullHardwareCycle();

        _motorC.setPower(0);

        waitOneFullHardwareCycle();

        RobotLog.d("moveForward::end");
    }

    public void raisearms(int distance, float power) throws InterruptedException {
        RobotLog.d("raisearms::begin");

        waitOneFullHardwareCycle();

        int currentPosition = _motorD.getCurrentPosition();

        RobotLog.d("raisearms::currentPosition " + currentPosition);

//        _motorD.setTargetPosition(distance + currentPosition);

//        waitOneFullHardwareCycle();

//        _motorD.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        waitOneFullHardwareCycle();

        _motorD.setPower(power);

        int sign = power >0 ? 1 : -1;

        int delta;

        do {
            waitOneFullHardwareCycle();

            int relativePosition = _motorD.getCurrentPosition() - currentPosition;

            delta = sign * (relativePosition - distance);

            RobotLog.d("raisearms::relativePosition " + relativePosition + " delta " + delta);

        } while (delta <-ENCODER_EPSILON);

        waitOneFullHardwareCycle();

        _motorD.setPower(0);

        waitOneFullHardwareCycle();

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

        waitOneFullHardwareCycle();

        _motorA.setPower(power);
        _motorB.setPower(power);

        int minDelta;

        int sign = power >0 ? 1 : -1;

        do {
            waitOneFullHardwareCycle();

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

        waitOneFullHardwareCycle();

        _motorA.setPower(0);
        _motorB.setPower(0);

        waitOneFullHardwareCycle();

        RobotLog.d("moveForward::end");
    }

    void dumpClimbers() throws InterruptedException {
        moveForward(convertInches(4), .25f);
        raisearms(17000, 1f);
        extendarms(6000, 1f);
        raisearms(8500, 1f);
        extendarms(3250, 1f);
        //        moveForward(convertInches(5), .25f);

        scooper(-.15f);
        sleep(500);
        scooper(0f);

        extendarms(-3250, -1f);
        raisearms(-7000, -1f);
        extendarms(-6000, -1f);
        raisearms(-17000, -1f);
        moveForward(convertInches(10), .25f);
    }
}
