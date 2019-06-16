package org.firstinspires.ftc.teamcode;



import android.graphics.Color;
import android.widget.Button;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.RobotLog;



import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.RobotLog;





import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;





/*

 * An example linear op mode where the pushbot

 * will drive in a square pattern using sleep()

 * and a for loop.

 */

public abstract class UnitBotMain extends LinearOpMode {

    protected DcMotor _frontLeft;

    protected DcMotor _frontRight;

    protected DcMotor _backLeft;

    protected DcMotor _backRight;

    protected DcMotor _motorIntake;
//
    protected DcMotor _motorStronkBoi;
//
    protected DcMotor _motorExpand1;
//
    protected DcMotor _motorExpand2;





    public DigitalChannel _extendLimit = null;

    public DigitalChannel _retractLimit = null;

    public DigitalChannel _button1 = null;

    public DigitalChannel _button2 = null;

    BNO055IMU _imu1;

    ColorSensor _sensorRGB;

    NormalizedColorSensor _sensorRGBArm;



    public Servo _servoHook = null;

    public Servo _servoReach1 = null;

    public Servo _servoReach2 = null;

    public Servo _servoGrab1 = null;

    public Servo _servoGrab2 = null;

    public Servo _servoPinchR = null;

    public Servo _servoPinchL = null;

    // we assume that the LED pin of the RGB sensor is connected to

    // digital port 5 (zero indexed).

    static final int RGB_LED_CHANNEL = 5;

    public AnalogInput _lightR;

    public AnalogInput _lightL;



    VuforiaLocalizer _vuforia;

    VuforiaTrackables _vuforiaTrackables;



    static final int BLUE_NEAR = 0;

    static final int RED_FAR = 1;

    static final int BLUE_FAR = 2;

    static final int RED_NEAR = 3;

    Orientation angles;

    float pitchAngle;

    float rollAngle;

    private int extend1 = 0;
    private int extend2 = 0;
    private int pivot = 0;
    private int lift = 0;


    public void initialize() throws InterruptedException {
        _frontLeft = hardwareMap.dcMotor.get("front_left");
        _frontLeft.setDirection(DcMotor.Direction.REVERSE);
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _frontRight = hardwareMap.dcMotor.get("front_right");
        _frontRight.setDirection(DcMotor.Direction.FORWARD);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _backLeft = hardwareMap.dcMotor.get("back_left");
        _backLeft.setDirection(DcMotor.Direction.REVERSE);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _backRight = hardwareMap.dcMotor.get("back_right");
        _backRight.setDirection(DcMotor.Direction.FORWARD);
        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorIntake = hardwareMap.dcMotor.get("motor_intake");
        _motorIntake.setDirection(DcMotor.Direction.REVERSE);
        _motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _motorStronkBoi = hardwareMap.dcMotor.get("strong_boi");
        _motorStronkBoi.setDirection(DcMotor.Direction.REVERSE);

        _motorStronkBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorStronkBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//
        _motorExpand1 = hardwareMap.dcMotor.get("motor_expand1");
//
        _motorExpand1.setDirection(DcMotor.Direction.REVERSE);
//
        _motorExpand1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
        _motorExpand2 = hardwareMap.dcMotor.get("motor_expand2");
//
        _motorExpand2.setDirection(DcMotor.Direction.REVERSE);
//
        _motorExpand2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        RobotLog.d("OctobotMain::initialize::_frontLeft::" + _frontLeft);

        RobotLog.d("OctobotMain::initialize::_frontRight::" + _frontRight);

        RobotLog.d("OctobotMain::initialize::_backLeft::" + _backLeft);

        RobotLog.d("OctobotMain::initialize::_backRight::" + _backRight);

        RobotLog.d("OctobotMain::initialize::_motorIntake::" + _motorIntake);
//
        RobotLog.d("OctobotMain::initialize::_motorLift::" + _motorExpand1);
//
        RobotLog.d("OctobotMain::initialize::_motorLift2::" + _motorExpand2);


        RobotLog.d("OctobotMain::initialize::initialize buttons");



        _button1 = hardwareMap.digitalChannel.get("button_1");

        _button1.setMode(DigitalChannel.Mode.INPUT);



        _button2 = hardwareMap.digitalChannel.get("button_2");

        _button2.setMode(DigitalChannel.Mode.INPUT);


        /*

            Initialize servos

         */



        RobotLog.d("OctobotMain::initialize::initialize servos");



        _servoHook = hardwareMap.servo.get("servo_hook");

        _servoGrab1 = hardwareMap.servo.get("servo_grab1");

        //_servoGrab2 = hardwareMap.servo.get("servo_grab2");

        //_servoReach1 = hardwareMap.servo.get("servo_extend1");

        //_servoReach2 = hardwareMap.servo.get("servo_extend2");

        _servoPinchL = hardwareMap.servo.get("servo_pinchl");

        _servoPinchR = hardwareMap.servo.get("servo_pinchr");


        _lightR = hardwareMap.analogInput.get("light_l");

        _lightL = hardwareMap.analogInput.get("light_r");


        /*

            Initialize IMU

         */



        // Set up the parameters with which we will use our IMU. Note that integration

        // algorithm here just reports accelerations to the logcat log; it doesn't actually

        // provide positional information.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        _imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
        _imu1.initialize(parameters);

        RobotLog.d("OctobotMain::initialize::end");

    }



    double getCurrentHeading() {
        Orientation angles = _imu1.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    double getCurrentPitch() {
        Orientation angles = _imu1.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        return angles.thirdAngle;
    }





    public void resetAllDriveMotorEncoders() throws InterruptedException {

        RobotControl.resetMotorEncoder(_frontLeft, this);

        RobotControl.resetMotorEncoder(_frontRight, this);

        RobotControl.resetMotorEncoder(_backLeft, this);

        RobotControl.resetMotorEncoder(_backRight, this);

    }

    public void resetAllNonDriveMotorEncoders() throws InterruptedException {

        RobotControl.resetMotorEncoder(_motorStronkBoi, this);

        RobotControl.resetMotorEncoder(_motorExpand1, this);

        RobotControl.resetMotorEncoder(_motorExpand2, this);

        RobotControl.resetMotorEncoder(_motorIntake, this);

    }



    public void runUsingEncoders() throws InterruptedException {

        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    public void runWithoutEncoders() throws InterruptedException {

        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    public void stopMotors() {

        _frontLeft.setPower(0);

        _frontRight.setPower(0);

        _backLeft.setPower(0);

        _backRight.setPower(0);

    }




    public void drive(DriverIF driver, boolean runUsingEncoders, boolean stopMotors) throws InterruptedException {
        RobotLog.d("DepotAuton::start");

        driver.start();

        RobotLog.d("UnitBotMain::drive()::resetAllDriveMotorEncoders()");

        resetAllDriveMotorEncoders();

        RobotLog.d("UnitBotMain::drive()::runUsingEncoders: " + runUsingEncoders);

        if (runUsingEncoders) {
            RobotLog.d("UnitBotMain::drive()::A");
            runUsingEncoders();
        } else {
            RobotLog.d("UnitBotMain::drive()::B");
            runWithoutEncoders();
        }

        boolean keepGoing = true;

        DriverIF.Steerage lastSteerage = null;

        long startTime = System.currentTimeMillis();

        int numSteeringUpdates = 0;

//        int lastPositionA = _frontLeft.getCurrentPosition();
//        int lastPositionB = _frontRight.getCurrentPosition();
//        int lastPositionC = _backLeft.getCurrentPosition();
//        int lastPositionD = _backRight.getCurrentPosition();
//        int lastPositionExtend1 = _motorExpand1.getCurrentPosition();
//        int lastPositionExtend2 = _motorExpand2.getCurrentPosition();
//
//        int keepPositionA = 1;
//        int keepPositionB = 1;
//        int keepPositionC = 1;
//        int keepPositionD = 1;
//        int keepPositionExtend1 = 1;
//        int keepPositionExtend2 = 1;
//
//
//        double lastPowerA = 0.0;
//        double lastPowerB = 0.0;
//        double lastPowerC = 0.0;
//        double lastPowerD = 0.0;
//        double lastPowerExtend1 = 0.0;
//        double lastPowerExtend2 = 0.0;

        while (keepGoing) {
            DriverIF.Steerage steerage = driver.getSteerage();

            if (lastSteerage == null || !steerage.equals(lastSteerage)) {
                ++numSteeringUpdates;

                RobotLog.d("UnitBotMain::drive()::steerage: " + steerage);

                double frontLeft = steerage.getLeft() - steerage.getStrafe();
                double frontRight = steerage.getRight() + steerage.getStrafe();
                double rearLeft = steerage.getLeft() + steerage.getStrafe();
                double rearRight = steerage.getRight() - steerage.getStrafe();

                double maxPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(rearLeft), Math.abs(rearRight)));

                if (maxPower > 1) {
                    frontLeft /= maxPower;
                    frontRight /= maxPower;
                    rearLeft /= maxPower;
                    rearRight /= maxPower;
                }

                RobotLog.d("UnitBotMain::drive()::power::frontRight: " + frontRight);
                RobotLog.d("UnitBotMain::drive()::power::rearRight: " + rearRight);
                RobotLog.d("UnitBotMain::drive()::power::frontLeft: " + frontLeft);
                RobotLog.d("UnitBotMain::drive()::power::rearLeft: " + rearLeft);

                _frontLeft.setPower(frontRight);
                _frontRight.setPower(rearRight);
                _backLeft.setPower(frontLeft);
                _backRight.setPower(rearLeft);

//                _motorExpand1.setPower(extend1);
//                _motorExpand2.setPower(extend2);

                lastSteerage = steerage;

                ++numSteeringUpdates;
            }

            int positionA = _frontLeft.getCurrentPosition();
            int positionB = _frontRight.getCurrentPosition();
            int positionC = _backLeft.getCurrentPosition();
            int positionD = _backRight.getCurrentPosition();

            RobotLog.d("UnitBotMain::drive()::positionA: " + positionA);
            RobotLog.d("UnitBotMain::drive()::positionB: " + positionB);
            RobotLog.d("UnitBotMain::drive()::positionC: " + positionC);
            RobotLog.d("UnitbotMain::drive()::positionD: " + positionD);

//
//            int positionExtend1 = _motorExpand1.getCurrentPosition();
//            int positionExtend2 = _motorExpand2.getCurrentPosition();


            /*
                If the change in encoder position is opposite what should have happened based on the power
                applied then don't use that encoder since it may be bad
             */

//            if (lastPowerA * (positionA - lastPositionA) < 0) {
//                keepPositionA = 0;
//            }
//
//
//            if (lastPowerB * (positionB - lastPositionB) < 0) {
//                keepPositionB = 0;
//            }
//
//            if (lastPowerC * (positionC - lastPositionC) < 0) {
//                keepPositionC = 0;
//            }
//
//            if (lastPowerD * (positionD - lastPositionD) < 0) {
//                keepPositionD = 0;
//            }
//
//
//            if (lastPowerExtend1 * (positionExtend1 - lastPositionExtend1) < 0) {
//                keepPositionExtend1 = 0;
//            }
//
//
//            if (lastPowerExtend2 * (positionExtend2 - lastPositionExtend2) < 0) {
//                keepPositionExtend2 = 0;
//            }

//            lastPositionA = positionA;
//            lastPositionB = positionB;
//            lastPositionC = positionC;
//            lastPositionD = positionD;
//
//            lastPositionExtend1 = positionExtend1;
//            lastPositionExtend2 = positionExtend2;
//
//            lastPowerA = _frontLeft.getPower();
//            lastPowerB = _frontRight.getPower();
//            lastPowerC = _backLeft.getPower();
//            lastPowerD = _backRight.getPower();
//            lastPositionExtend1 = (int) _motorExpand1.getPower();
//            lastPositionExtend2 = (int) _motorExpand2.getPower();

//            if (keepPositionA + keepPositionB + keepPositionC + keepPositionD != 0) {
//                int position = (Math.abs(positionA) * keepPositionA + Math.abs(positionB) * keepPositionB + Math.abs(positionC) * keepPositionC + Math.abs(positionD) * keepPositionD) /
//
//                        (keepPositionA + keepPositionB + keepPositionC + keepPositionD);
//
//                RobotLog.d("UnitbotMain::drive()::position: " + position);
//
//                keepGoing = driver.keepGoing(position);
//            } else {
//                keepGoing = false;
//            }

//            if (keepPositionA + keepPositionB + keepPositionC + keepPositionD != 0) {
//                int position = (Math.abs(positionA) * keepPositionA + Math.abs(positionB) * keepPositionB + Math.abs(positionC) * keepPositionC + Math.abs(positionD) * keepPositionD) /
//
//                        (keepPositionA + keepPositionB + keepPositionC + keepPositionD);
//
//                RobotLog.d("UnitbotMain::drive()::position: " + position);
//
//            } else {
//                keepGoing = false;
//            }

            keepGoing = driver.keepGoing( (Math.abs(positionA) + Math.abs(positionB) + Math.abs(positionC) + Math.abs(positionD) ) / 4 );

            RobotLog.d("UnitbotMain::drive()::keepGoing: " + keepGoing);

            idle();
        }

        if (stopMotors) {
            stopMotors();
        }

        driver.finish();
    }



    public void turn(TurnerIF turner, boolean runUsingEncoders, boolean stopMotors) throws InterruptedException {

        turner.start();



        if (runUsingEncoders) {

            runUsingEncoders();

        } else {

            runWithoutEncoders();

        }



        boolean keepGoing = true;



        double lastScaleFactor = 0;

        double lastPower = 0;



        while (keepGoing) {

            int positionA = _frontLeft.getCurrentPosition();

            int positionB = _frontRight.getCurrentPosition();

            int positionC = _backLeft.getCurrentPosition();

            int positionD = _backRight.getCurrentPosition();



            RobotLog.d("UnitBotMain::turn()::_frontLeftPosition: " + positionA);

            RobotLog.d("UnitBotMain::turn()::_frontRightPosition: " + positionB);

            RobotLog.d("UnitBotMain::turn()::_backLeftPosition: " + positionC);

            RobotLog.d("UnitbotMain::turn()::_backRightPosition: " + positionD);



//            waitForNextHardwareCycle();



            double power = turner.getPower();

            double scaleFactor = turner.getScaleFactor();



            RobotLog.d("UnitBotMain::turn()::power: " + power);

            RobotLog.d("UnitBotMain::turn()::scaleFactor: " + scaleFactor);



            keepGoing = turner.keepGoing(0);



            RobotLog.d("UnitBotMain::turn()::keepGoing: " + keepGoing);



            if (keepGoing && (power != lastPower || scaleFactor != lastScaleFactor)) {

                if (Double.isNaN(scaleFactor)) {

                    keepGoing = false;

                } else {

                    RobotLog.d("UnitBotMain::turn()::power * scaleFactor: " + power * scaleFactor);



                    _frontLeft.setPower(-power * scaleFactor);

                    _frontRight.setPower(power * scaleFactor);

                    _backLeft.setPower(-power * scaleFactor);

                    _backRight.setPower(power * scaleFactor);

                    RobotLog.d("UnitBotMain::turn()::_frontLeftPower: " + _frontLeft.getPower());
                    RobotLog.d("UnitBotMain::turn()::_frontRightPower: " + _frontRight.getPower());
                    RobotLog.d("UnitBotMain::turn()::_backLeftPower: " + _backLeft.getPower());
                    RobotLog.d("UnitBotMain::turn()::_backRightPower: " + _backRight.getPower());

                }



                lastPower = power;

                lastScaleFactor = scaleFactor;

            }



            Thread.sleep(1);



        }



        if (stopMotors) {

            stopMotors();

        }

    }



    public void balance() {

        float leftX = gamepad1.left_stick_x;

        float leftY;



        float rightX = gamepad1.right_stick_x;

        float rightY;



        float multiplier;



        angles = _imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        pitchAngle = 0;



        if ((angles.thirdAngle) > 0) {

            pitchAngle = 180 - angles.thirdAngle;

        } else if (angles.thirdAngle < 0) {

            pitchAngle = angles.thirdAngle + 180;

            pitchAngle = -pitchAngle;

        }

        telemetry.addData("pitch", pitchAngle);



        rollAngle = -angles.secondAngle;



        multiplier = .01f;



        telemetry.addData("roll", angles.secondAngle);



        leftY = multiplier * pitchAngle;

        rightY = multiplier * pitchAngle;



        leftX = multiplier * rollAngle;

        rightX = multiplier * rollAngle;



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



        double motorCPower = RobotControl.convertStickToPower(rearLeft);

        double motorAPower = RobotControl.convertStickToPower(frontLeft);

        double motorBPower = RobotControl.convertStickToPower(frontRight);

        double motorDPower = RobotControl.convertStickToPower(rearRight);


        RobotLog.d("Motor Power " + motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);

        telemetry.addData("Motor Power", motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);



        _frontLeft.setPower(motorAPower);

        _frontRight.setPower(motorBPower);

        _backLeft.setPower(motorCPower);

        _backRight.setPower(motorDPower);

    }



    public void waitUntilStopped(StopperIF stopper) throws InterruptedException {

        stopper.start();



        while (stopper.keepGoing(0)) {

            Thread.sleep(1);

        }



        stopper.finish();

    }

    public boolean transition(DcMotor _motorStronkBoi, DcMotor _motorExpand1, DcMotor _motorExpand2, Servo _servoGrab1, Servo _servoGrab2, Servo _servoReach1, Servo _servoReach2){

        _servoGrab1.setPosition(0);
        _servoGrab2.setPosition(0);

        _servoReach1.setPosition(0);
        _servoReach2.setPosition(0);

        if(_motorIntake.getCurrentPosition() < -4500){
            while(_motorIntake.getCurrentPosition() < -4500){
                _motorExpand1.setPower(1);
                _motorExpand2.setPower(-1);
            }
        }else if(_motorIntake.getCurrentPosition() > -4500){
            while(_motorIntake.getCurrentPosition() > -4500){
                _motorExpand1.setPower(-1);
                _motorExpand2.setPower(1);
            }
        }else{
            _motorExpand1.setPower(0);
            _motorExpand2.setPower(0);
        }

        _motorStronkBoi.setTargetPosition(-650);

        return true;
    }

    public boolean intake(DcMotor _motorStronkBoi, DcMotor _motorExpand1, DcMotor _motorExpand2, Servo _servoGrab1, Servo _servoGrab2, Servo _servoReach1, Servo _servoReach2){

        _servoGrab1.setPosition(0);
        _servoGrab2.setPosition(0);

        _servoReach1.setPosition(0);
        _servoReach2.setPosition(0);

        if(_motorIntake.getCurrentPosition() < -21000){
            while(_motorIntake.getCurrentPosition() < -21000){
                _motorExpand1.setPower(1);
                _motorExpand2.setPower(-1);
            }
        }else if(_motorIntake.getCurrentPosition() > 21000){
            while(_motorIntake.getCurrentPosition() > 21000){
                _motorExpand1.setPower(-1);
                _motorExpand2.setPower(1);
            }
        }else{
            _motorExpand1.setPower(0);
            _motorExpand2.setPower(0);
        }

        _motorStronkBoi.setTargetPosition(0);

        return true;
    }

    public boolean deposit(DcMotor _motorStronkBoi, DcMotor _motorExpand1, DcMotor _motorExpand2, Servo _servoGrab1, Servo _servoGrab2, Servo _servoReach1, Servo _servoReach2, DigitalChannel _button1, DigitalChannel _button2){

        _servoGrab1.setPosition(1);
        _servoGrab2.setPosition(1);

        boolean is1 = _button1.getState();
        boolean is2 = _button2.getState();

        if(is1){
            _servoReach1.setPosition(1);
        }else{
            _servoReach1.setPosition(.5);
        }

        if(is2){
            _servoReach2.setPosition(1);
        }else{
            _servoReach2.setPosition(.5);
        }


        if(_motorIntake.getCurrentPosition() < -19000){
            while(_motorIntake.getCurrentPosition() < -19000){
                _motorExpand1.setPower(1);
                _motorExpand2.setPower(-1);
            }
        }else if(_motorIntake.getCurrentPosition() > -19000){
            while(_motorIntake.getCurrentPosition() > -19000){
                _motorExpand1.setPower(-1);
                _motorExpand2.setPower(1);
            }
        }else{
            _motorExpand1.setPower(0);
            _motorExpand2.setPower(0);
        }

        _motorStronkBoi.setTargetPosition(-2600);

        return true;
    }

    public boolean hang(DcMotor _motorStronkBoi, DcMotor _motorExpand1, DcMotor _motorExpand2, Servo _servoGrab1, Servo _servoGrab2, Servo _servoReach1, Servo _servoReach2, Servo _servoHook){

        _servoGrab1.setPosition(0);
        _servoGrab2.setPosition(0);

        _servoReach1.setPosition(0);
        _servoReach2.setPosition(0);

        if(_motorIntake.getCurrentPosition() < -500){
            while(_motorIntake.getCurrentPosition() < -500){
                _motorExpand1.setPower(1);
                _motorExpand2.setPower(-1);
            }
        }else if(_motorIntake.getCurrentPosition() > -500){
            while(_motorIntake.getCurrentPosition() > -500){
                _motorExpand1.setPower(-1);
                _motorExpand2.setPower(1);
            }
        }else{
            _motorExpand1.setPower(0);
            _motorExpand2.setPower(0);
        }

        _servoHook.setPosition(1);

        _motorStronkBoi.setTargetPosition(-2600);

        return true;
    }

    public boolean end(DcMotor _motorStronkBoi, DcMotor _motorExpand1, DcMotor _motorExpand2, Servo _servoGrab1, Servo _servoGrab2, Servo _servoReach1, Servo _servoReach2, Servo _servoHook){

        _servoGrab1.setPosition(0);
        _servoGrab2.setPosition(0);

        _servoReach1.setPosition(0);
        _servoReach2.setPosition(0);

        if(_motorIntake.getCurrentPosition() < -500){
            while(_motorIntake.getCurrentPosition() < -500){
                _motorExpand1.setPower(1);
                _motorExpand2.setPower(-1);
            }
        }else if(_motorIntake.getCurrentPosition() > -500){
            while(_motorIntake.getCurrentPosition() > -500){
                _motorExpand1.setPower(-1);
                _motorExpand2.setPower(1);
            }
        }else{
            _motorExpand1.setPower(0);
            _motorExpand2.setPower(0);
        }

        _servoHook.setPosition(0);

//        _motorStronkBoi.setTargetPosition(0);

        return true;
    }

    public boolean attacc(DcMotor _motorStronkBoi, DcMotor _motorExpand1, DcMotor _motorExpand2, Servo _servoGrab1, Servo _servoGrab2, Servo _servoReach1, Servo _servoReach2){

        _servoGrab1.setPosition(0);
        _servoGrab2.setPosition(0);

        _servoReach1.setPosition(0);
        _servoReach2.setPosition(0);

        if(_motorIntake.getCurrentPosition() < -500){
            while(_motorIntake.getCurrentPosition() < -500){
                _motorExpand1.setPower(1);
                _motorExpand2.setPower(-1);
            }
        }else if(_motorIntake.getCurrentPosition() > -500){
            while(_motorIntake.getCurrentPosition() > -500){
                _motorExpand1.setPower(-1);
                _motorExpand2.setPower(1);
            }
        }else{
            _motorExpand1.setPower(0);
            _motorExpand2.setPower(0);
        }

        _motorStronkBoi.setTargetPosition(0);

        return true;
    }

    public void extendServo(Servo _servoReach1, Servo _servoReach2, Servo _servoGrab1, Servo _servoGrab2, DigitalChannel _button1, DigitalChannel _button2){
        _servoGrab1.setPosition(0);
        _servoGrab2.setPosition(0);

        if(_button1.getState() == true){
            _servoReach1.setPosition(1);
        }else{
            _servoReach1.setPosition(.5);
        }

        if(_button2.getState() == true){
            _servoReach2.setPosition(1);
        }else{
            _servoReach2.setPosition(.5);
        }
    }

    public void retractServo(Servo _servoReach1, Servo _servoReach2, Servo _servoGrab1, Servo _servoGrab2){
        _servoReach1.setPosition(0);
        _servoReach2.setPosition(0);

        _servoGrab1.setPosition(0);
        _servoGrab2.setPosition(0);
    }

    public void pivot(int distance){
        //negative open, positive close

        int mvalue;
        int tvalue;
        int count = 0;
        boolean isPos = distance > 0;

        _motorStronkBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mvalue = _motorStronkBoi.getCurrentPosition();

        tvalue = mvalue + distance;

        _motorStronkBoi.setPower(isPos ? 1 : -1);

        while((isPos ? _motorStronkBoi.getCurrentPosition() < tvalue : _motorStronkBoi.getCurrentPosition() > tvalue) && count < 400){
            sleep(5);
            _servoGrab1.setPosition(.3);
            count++;
            telemetry.addData("Encoder: ", _motorStronkBoi.getCurrentPosition());
            telemetry.addData("Count: ", count);
            telemetry.update();
        }

        _motorStronkBoi.setPower(1);

        _motorStronkBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        count = 0;
        while(count < 5) {
            _motorStronkBoi.setTargetPosition(tvalue);
            count ++;
        }
    }
    public void extend(int distance){
        //negative in, positive out

        distance *= -1;

        int mvalue;
        int tvalue;
        int count = 0;
        boolean isPos = distance > 0;

        _motorExpand1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorExpand2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mvalue = _motorExpand1.getCurrentPosition();

        tvalue = mvalue + distance;

        _motorExpand1.setPower(isPos ? .7 : -.7);
        _motorExpand2.setPower(isPos ? -.7 : .7);

        while((isPos ? _motorExpand1.getCurrentPosition() < tvalue : _motorExpand1.getCurrentPosition() > tvalue) && count < 250){
            sleep(5);
            _servoGrab1.setPosition(.3);
            count++;
            telemetry.addData("Encoder: ", _motorExpand1.getCurrentPosition());
            telemetry.addData("Count: ", count);
            telemetry.update();
        }

        _motorExpand1.setPower(0);
        _motorExpand2.setPower(0);
    }

    public void initializeCV(GoldAlignDetector detector){
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();
        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
        telemetry.addData("X Pos" , detector.getXPosition());
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        // Optional Tuning
        detector.alignSize = 20; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
    }

    public void multiple(int distancePivot, int distanceExtend){
        //negative open, positive close
        //negative in, positive out
        int mvalue;
        int tvalue;
        int count = 0;
        boolean isPos = distancePivot > 0;

        distanceExtend *= -1;

        _motorStronkBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mvalue = _motorStronkBoi.getCurrentPosition();

        tvalue = mvalue + distancePivot;

        int mvalue2;
        int tvalue2;
        int count2 = 0;
        boolean isPos2 = distanceExtend > 0;

        _motorExpand1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorExpand2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mvalue2 = _motorExpand1.getCurrentPosition();

        tvalue2 = mvalue2 + distanceExtend;

        _motorStronkBoi.setPower(isPos ? 1 : -1);
        _motorExpand1.setPower(isPos2 ? .5 : -.5);
        _motorExpand2.setPower(isPos2 ? -.5 : 5);

        boolean pivotGo = true;
        boolean extendGo = true;

        while((pivotGo || extendGo) && count < 400){
            sleep(1);
            _servoGrab1.setPosition(.3);
            count++;
            telemetry.addData("Encoder Pivot: ", _motorStronkBoi.getCurrentPosition());
            telemetry.addData("Encoder Extend: ",_motorExpand1.getCurrentPosition());
            telemetry.addData("Count: ", count);

            if(!(isPos ? _motorStronkBoi.getCurrentPosition() < tvalue : _motorStronkBoi.getCurrentPosition() > tvalue)){
                _motorStronkBoi.setPower(1);
                _motorStronkBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                _motorStronkBoi.setTargetPosition(tvalue);
                pivotGo = false;
                telemetry.addLine("Pivot Done");

            }
            if(!(isPos2 ? _motorExpand1.getCurrentPosition() < tvalue2 : _motorExpand1.getCurrentPosition() > tvalue2)){
                _motorExpand1.setPower(0);
                _motorExpand2.setPower(0);
                extendGo = false;
                telemetry.addLine("Extend Done");
            }
            telemetry.update();
        }
    }

}