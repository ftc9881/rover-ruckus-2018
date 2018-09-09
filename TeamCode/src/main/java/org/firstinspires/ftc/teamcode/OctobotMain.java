package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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


import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;


/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep()
 * and a for loop.
 */
public abstract class OctobotMain extends LinearOpMode
{
    protected DcMotor _motorA;
    protected DcMotor _motorB;
    protected DcMotor _motorC;
    protected DcMotor _motorD;
    protected DcMotor _motorSlide;
    protected DcMotor _motorLift;
    protected DcMotor _motorSpinner;
    protected DcMotor _motorScissor;


    public DigitalChannel _frontSliderLimit = null;
    public DigitalChannel _rearSliderLimit = null;
    public DigitalChannel _button2 = null;
    public DigitalChannel _button3 = null;

//    BNO055IMU _imu;
    BNO055IMU _imu1;
//    BNO055IMU _imu2;


    //DeviceInterfaceModule _cdim;
    ColorSensor _sensorRGB;
    NormalizedColorSensor _sensorRGBArm;

    public Servo _servoA = null;
    public Servo _servoB = null;
    public Servo _servoC = null;
    public Servo _servoD = null;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int RGB_LED_CHANNEL = 5;

    VuforiaLocalizer _vuforia;
    VuforiaTrackables _vuforiaTrackables;

    static final int BLUE_NEAR = 0;
    static final int RED_FAR = 1;
    static final int BLUE_FAR = 2;
    static final int RED_NEAR = 3;

    Orientation angles;
    float pitchAngle;
    float rollAngle;

    /*
        IR Sensorsf
     */

    SharpDistanceSensor _irSensorLeft;
    SharpDistanceSensor _irSensorRight;

    /*
        Sonar sensors
     */

//    public MaxSonarI2CXL _sonarLeft;
//    public MaxSonarI2CXL _sonarRight;
//    public SonarArrayManager _sonarArrayManager;

    /**
     * Initialize all sensors, motors, etc.
     * @throws InterruptedException
     */
    public void initialize() throws InterruptedException {
        _motorA = hardwareMap.dcMotor.get("motor_a");
        _motorA.setDirection(DcMotor.Direction.REVERSE);
        _motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorB = hardwareMap.dcMotor.get("motor_b");
        _motorB.setDirection(DcMotor.Direction.REVERSE);
        _motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorC = hardwareMap.dcMotor.get("motor_c");
        _motorC.setDirection(DcMotor.Direction.FORWARD);
        _motorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorD = hardwareMap.dcMotor.get("motor_d");
        _motorD.setDirection(DcMotor.Direction.FORWARD);
        _motorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorSlide = hardwareMap.dcMotor.get("motor_slide");
        _motorSlide.setDirection(DcMotor.Direction.REVERSE);
        _motorSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorLift = hardwareMap.dcMotor.get("motor_lift");
        _motorLift.setDirection(DcMotor.Direction.REVERSE);
        _motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorSpinner = hardwareMap.dcMotor.get("motor_spinner");
        _motorSpinner.setDirection(DcMotor.Direction.REVERSE);
        _motorSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorScissor = hardwareMap.dcMotor.get("motor_scissor");
        _motorScissor.setDirection(DcMotor.Direction.REVERSE);
        _motorScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        RobotControl.resetAllDriveMotorEncoders(_motorA, _motorB, _motorC, _motorD, this);

        RobotLog.d("OctobotMain::initialize::_motorA::" + _motorA);
        RobotLog.d("OctobotMain::initialize::_motorB::" + _motorB);
        RobotLog.d("OctobotMain::initialize::_motorC::" + _motorC);
        RobotLog.d("OctobotMain::initialize::_motorD::" + _motorD);
        RobotLog.d("OctobotMain::initialize::_motorSlide::" + _motorSlide);
        RobotLog.d("OctobotMain::initialize::_motorLift::" + _motorLift);
        RobotLog.d("OctobotMain::initialize::_motorSpinner::" + _motorSpinner);
        RobotLog.d("OctobotMain::initialize::_motorScissor::" + _motorScissor);

        /*
            Initialize pusher button
         */

        RobotLog.d("OctobotMain::initialize::initialize buttons");

        _frontSliderLimit = hardwareMap.digitalChannel.get("button_0");
        _frontSliderLimit.setMode(DigitalChannel.Mode.INPUT);

        _rearSliderLimit = hardwareMap.digitalChannel.get("button_1");
        _rearSliderLimit.setMode(DigitalChannel.Mode.INPUT);

        _button2 = hardwareMap.digitalChannel.get("button_2");
        _button2.setMode(DigitalChannel.Mode.INPUT);

        _button3 = hardwareMap.digitalChannel.get("button_3");
        _button3.setMode(DigitalChannel.Mode.INPUT);

        /*
            Initialize servos
         */

        RobotLog.d("OctobotMain::initialize::initialize servos");

        _servoA = hardwareMap.servo.get("servo_top");
        _servoB = hardwareMap.servo.get("servo_bottom");
        _servoC = hardwareMap.servo.get("servo_arm");
        _servoD = hardwareMap.servo.get("servo_relic");

        /*
            Initialize IMU
         */

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
//        _imu = hardwareMap.get(BNO055IMU.class, "imu");
        _imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
//        _imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
//
//        RobotLog.d("OctobotMain::initialize::initialize imu");
//
//        _imu.initialize(parameters);
//
//        RobotLog.d("OctobotMain::initialize::initialize imu1");
//
        _imu1.initialize(parameters);
//
//        RobotLog.d("OctobotMain::initialize::initialize imu2");
//
//        _imu2.initialize(parameters);

        /*
            Initialize RGB sensor
        */

        RobotLog.d("OctobotMain::initialize::initialize color sensor");

        _sensorRGB = hardwareMap.colorSensor.get("color_sensor");
//        _sensorRGB = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        _sensorRGBArm = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_2");

        /*
            IR Sensors
         */

        _irSensorLeft = new SharpDistanceSensor(hardwareMap.analogInput.get("ir_left"),
                0.061617183,
                -0.008870024,
                3.398418633
        );

        _irSensorRight = new SharpDistanceSensor(hardwareMap.analogInput.get("ir_right"),
                0.049154276,
                -0.009550867,
                7.419945783
        );

        /*
            Sonar Sensors
         */

//        _sonarLeft = hardwareMap.get(MaxSonarI2CXL.class, "sonar_left");
//        _sonarRight = hardwareMap.get(MaxSonarI2CXL.class, "sonar_right");
//
//        _sonarLeft.setI2cAddress(I2cAddr.create8bit(0xE0));
//        _sonarRight.setI2cAddress(I2cAddr.create8bit(0xDE));
//
//        _sonarArrayManager = new SonarArrayManager();
//        _sonarArrayManager.addSonar("left", _sonarLeft);
//        _sonarArrayManager.addSonar("right", _sonarRight);

        RobotLog.d("OctobotMain::initialize::end");
    }

    double getCurrentHeading() {
        Orientation angles = _imu1.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /*
        Iintialize Vuforia library, beacons and camera position
     */

    void initializeVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQMmXDH/////AAAAGXvAv3V7xURDoUrFi6n/qG0LC5kmnnRDMaBE2DbHd3pUuUoMWpX4gA0ZWWxCDoU5Fd189JV3/q1hYeTJGgSschiTneHnBGjCp3KK+ADOnwyxeu8odM4s/CFViozG+D8yyt6sORwv3yWK4g05DOP0dMpUdkUVuADqRUoIYPex/dkJajOXnhteO66gPxZOMCApNuTue1pyYueBeOIGWYC2DY7XQP2DeEL1BssxiXupkFtOsQWE6FWeKvKRwKdTmDQymM+s5SRfhxF/8Yd85keIe0jL/OyzeSIpq8RnHOGz2HjTEQK9UxWD/+A8jy584gdb8X3613zXPURYGKhU657+asjXTb1Yg8smo8CRAUcpEE24";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.cameraMonitorFeedback = AXES; //set camera feedback icon (An axis is placed on photo in preview)
        parameters.useExtendedTracking = false;

        _vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        _vuforiaTrackables = _vuforia.loadTrackablesFromAsset("RelicVuMark");
    }

    void startVuforia() {
        _vuforiaTrackables.activate();
    }

    void stopVuforia() {
        _vuforiaTrackables.deactivate();
    }

    RelicRecoveryVuMark getDisplayedVuMark() {
        VuforiaTrackable vuforiaTemplate = _vuforiaTrackables.get(0);

        return RelicRecoveryVuMark.from(vuforiaTemplate);
    }

    OpenGLMatrix getVuMarkPose() {
        VuforiaTrackable vuforiaTemplate = _vuforiaTrackables.get(0);
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) vuforiaTemplate.getListener()).getPose();

        return pose;
    }


    public void resetAllDriveMotorEncoders() throws InterruptedException {
        RobotControl.resetMotorEncoder(_motorA, this);
        RobotControl.resetMotorEncoder(_motorB, this);
        RobotControl.resetMotorEncoder(_motorC, this);
        RobotControl.resetMotorEncoder(_motorD, this);
    }

    public void runUsingEncoders() throws InterruptedException {
        _motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoders() throws InterruptedException {
        _motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _motorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _motorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopMotors() {
        _motorA.setPower(0);
        _motorB.setPower(0);
        _motorC.setPower(0);
        _motorD.setPower(0);
    }

    public void resetNonDriveMotorEncoders() throws InterruptedException {
        RobotControl.resetMotorEncoder(_motorLift, this);
    }

    public void runNonDriveUsingEncoders() throws InterruptedException {
        _motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runNonDriveWithoutEncoders() throws InterruptedException {
        _motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(DriverIF driver, boolean runUsingEncoders, boolean stopMotors) throws InterruptedException {
//        RobotLog.d("OctobotMain::drive()::A");
        driver.start();

//        RobotLog.d("OctobotMain::drive()::B");

        resetAllDriveMotorEncoders();

        if(runUsingEncoders) {
            runUsingEncoders();
        }
        else {
            runWithoutEncoders();
        }

//        RobotLog.d("OctobotMain::drive()::C");

        boolean keepGoing = true;

        DriverIF.Steerage lastSteerage = null;

        long startTime = System.currentTimeMillis();
        int numSteeringUpdates = 0;

//        RobotLog.d("OctobotMain::drive()::D");

        int lastPositionA = _motorA.getCurrentPosition();
        int lastPositionB = _motorB.getCurrentPosition();
        int lastPositionC = _motorC.getCurrentPosition();
        int lastPositionD = _motorD.getCurrentPosition();

//        RobotLog.d("OctobotMain::drive()::initial positionA: " + lastPositionA);
//        RobotLog.d("OctobotMain::drive()::initial positionB: " + lastPositionB);
//        RobotLog.d("OctobotMain::drive()::initial positionC: " + lastPositionC);
//        RobotLog.d("OctobotMain::drive()::initial positionD: " + lastPositionD);

        int keepPositionA = 1;
        int keepPositionB = 1;
        int keepPositionC = 1;
        int keepPositionD = 1;

        double lastPowerA = 0;
        double lastPowerB = 0;
        double lastPowerC = 0;
        double lastPowerD = 0;

        while (keepGoing) {
            DriverIF.Steerage steerage = driver.getSteerage();

            if(lastSteerage == null || !steerage.equals(lastSteerage)) {
                ++numSteeringUpdates;

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

                _motorA.setPower(frontRight);
                _motorB.setPower(rearRight);
                _motorC.setPower(frontLeft);
                _motorD.setPower(rearLeft);

//                RobotLog.d("OctobotMain::drive()::frontRight: " + frontRight);
//                RobotLog.d("OctobotMain::drive()::rearRight: " + rearRight);
//                RobotLog.d("OctobotMain::drive()::frontLeft: " + frontLeft);
//                RobotLog.d("OctobotMain::drive()::rearLeft: " + rearLeft);
            }

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

//            RobotLog.d("OctobotMain::drive()::positionA: " + positionA);
//            RobotLog.d("OctobotMain::drive()::positionB: " + positionB);
//            RobotLog.d("OctobotMain::drive()::positionC: " + positionC);
//            RobotLog.d("OctobotMain::drive()::positionD: " + positionD);

            /*
                If the change in encoder position is opposite what should have happened based on the power
                applied then don't use that encoder since it may be bad
             */

            if(lastPowerA * (positionA - lastPositionA) < 0) {
                keepPositionA = 0;
//                RobotLog.d("OctobotMain::drive()::lastPowerA * (positionA - lastPositionA): " + (lastPowerA * (positionA - lastPositionA)));
            }

            if(lastPowerB * (positionB - lastPositionB) < 0) {
                keepPositionB = 0;
//                RobotLog.d("OctobotMain::drive()::lastPowerB * (positionB - lastPositionB): " + (lastPowerB * (positionB - lastPositionB)));
            }

            if(lastPowerC * (positionC - lastPositionC) < 0) {
                keepPositionC = 0;
//                RobotLog.d("OctobotMain::drive()::lastPowerC * (positionC - lastPositionC): " + (lastPowerC * (positionC - lastPositionC)));
            }

            if(lastPowerD * (positionD - lastPositionD) < 0) {
                keepPositionD = 0;
//                RobotLog.d("OctobotMain::drive()::lastPowerD * (positionD - lastPositionD): " + (lastPowerD * (positionD - lastPositionD)));
            }

            lastPositionA = positionA;
            lastPositionB = positionB;
            lastPositionC = positionC;
            lastPositionD = positionD;

            lastPowerA = _motorA.getPower();
            lastPowerB = _motorB.getPower();
            lastPowerC = _motorC.getPower();
            lastPowerD = _motorD.getPower();

//            RobotLog.d("OctobotMain::drive()::keepPositionA: " + keepPositionA);
//            RobotLog.d("OctobotMain::drive()::keepPositionB: " + keepPositionB);
//            RobotLog.d("OctobotMain::drive()::keepPositionC: " + keepPositionC);
//            RobotLog.d("OctobotMain::drive()::keepPositionD: " + keepPositionD);

            if(keepPositionA + keepPositionB + keepPositionC + keepPositionD != 0) {
                int position = (Math.abs(positionA) * keepPositionA + Math.abs(positionB) * keepPositionB + Math.abs(positionC) * keepPositionC + Math.abs(positionD) * keepPositionD) /
                        (keepPositionA + keepPositionB + keepPositionC + keepPositionD);

//                RobotLog.d("OctobotMain::drive()::position: " + position);
//
//                RobotLog.d("OctobotMain::drive()::G");

                keepGoing = driver.keepGoing(position);

//                RobotLog.d("OctobotMain::drive()::keepGoing: " + keepGoing);
            }
            else {
//                RobotLog.d("OctobotMain::drive()::No valid position");
                keepGoing = false;
            }

            idle();
        }

//        RobotLog.d("OctobotMain::drive::steering update interval: " + (System.currentTimeMillis() - startTime) / numSteeringUpdates);

        if(stopMotors) {
            stopMotors();
        }

//        RobotLog.d("OctobotMain::drive()::I");

        driver.finish();

//        RobotLog.d("OctobotMain::drive()::J");

    }

    public void turn(TurnerIF turner, boolean runUsingEncoders, boolean stopMotors) throws InterruptedException {
        turner.start();

        if(runUsingEncoders) {
            runUsingEncoders();
        }
        else {
            runWithoutEncoders();
        }

        boolean keepGoing = true;

        double lastScaleFactor = 0;
        double lastPower = 0;

        while(keepGoing) {
            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("OctobotMain::turn()::positionA: " + positionA);
            RobotLog.d("OctobotMain::turn()::positionB: " + positionB);
            RobotLog.d("OctobotMain::turn()::positionC: " + positionC);
            RobotLog.d("OctobotMain::turn()::positionD: " + positionD);

//            waitForNextHardwareCycle();

            double power = turner.getPower();
            double scaleFactor = turner.getScaleFactor();

            RobotLog.d("OctobotMain::turn()::power: " + power);
            RobotLog.d("OctobotMain::turn()::scaleFactor: " + scaleFactor);

            keepGoing = turner.keepGoing(0);

            RobotLog.d("OctobotMain::turn()::keepGoing: " + keepGoing);

            if (keepGoing && (power != lastPower || scaleFactor != lastScaleFactor)) {
                if (Double.isNaN(scaleFactor)) {
                    keepGoing = false;
                } else {
                    RobotLog.d("OctobotMain::turn()::power * scaleFactor: " + power * scaleFactor);

                    _motorA.setPower(-power * scaleFactor);
                    _motorB.setPower(-power * scaleFactor);
                    _motorC.setPower(power * scaleFactor);
                    _motorD.setPower(power * scaleFactor);
                }

                lastPower = power;
                lastScaleFactor = scaleFactor;
            }

            Thread.sleep(1);

//            idle();
//            try {
//                Thread.yield();
//            }
//            catch(Throwable t) {
//            }
        }

        if(stopMotors) {
            stopMotors();
        }
    }
    public void balance(){
        float leftX = gamepad1.left_stick_x;
        float leftY;

        float rightX = gamepad1.right_stick_x;
        float rightY;

        float multiplier;

        angles   = _imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        pitchAngle = 0;

        if ((angles.thirdAngle) > 0){
            pitchAngle = 180-angles.thirdAngle;
        }
        else if(angles.thirdAngle < 0){
            pitchAngle = angles.thirdAngle + 180;
            pitchAngle = -pitchAngle;
        }
        telemetry.addData("pitch",pitchAngle);

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

        double motorAPower = RobotControl.convertStickToPower(rearLeft);
        double motorBPower = RobotControl.convertStickToPower(frontLeft);
        double motorCPower = RobotControl.convertStickToPower(frontRight);
        double motorDPower = RobotControl.convertStickToPower(rearRight);

        RobotLog.d("Motor Power " + motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);
        telemetry.addData("Motor Power", motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);

        _motorA.setPower(motorAPower);
        _motorB.setPower(motorBPower);
        _motorC.setPower(motorCPower);
        _motorD.setPower(motorDPower);
    }

//    public Servo getGrabberServo() {
//        float[] hsvValuesGrabber = new float[3];
//
////        NormalizedRGBA colorsGrabber = _sensorRGB.getNormalizedColors();
////        Color.colorToHSV(colorsGrabber.toColor(), hsvValuesGrabber);
//
//        Color.RGBToHSV((_sensorRGB.red() * 255) / 800, (_sensorRGB.green() * 255) / 800, (_sensorRGB.blue() * 255) / 800, hsvValuesGrabber);
//
//        float hueGrabber = hsvValuesGrabber[0];
//        float saturationGrabber = hsvValuesGrabber[1];
//
//        RobotLog.d("OctobotMain::getGrabberServo()::hueGrabber: " + hueGrabber);
//        RobotLog.d("OctobotMain::getGrabberServo()::saturationGrabber: " + saturationGrabber);
//
//        boolean isBlue = true;
//
//        if (hueGrabber >= 0 && hueGrabber <= 90) {
//            isBlue = false;
//        } else if (hueGrabber <= 260 && hueGrabber >= 120) {
//            isBlue = true;
//        }
//
//        RobotLog.d("OctobotMain::getGrabberServo()::isBlue: " + isBlue);
//
//        if(!isBlue){
//            return _servoGrabberBlue;
//        }
//        else{
//            return _servoGrabberRed;
//        }
   // }

    public RelicRecoveryVuMark getVuforia(){
        int count = 0;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        while(vuMark == RelicRecoveryVuMark.UNKNOWN && count < 400){
            vuMark = getDisplayedVuMark();
            sleep(5);
            count += 1;
        }
        if(vuMark == RelicRecoveryVuMark.UNKNOWN){
            vuMark  = RelicRecoveryVuMark.CENTER;
        }
        return vuMark;
    }

    public NormalizedRGBA getColorsRobust(NormalizedColorSensor colorSensor) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float[] hsvValuesArm = new float[3];

        int counter = 0;

        while(counter < 10 && hsvValuesArm[1] < .5) {
            colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValuesArm);
            sleep(10);
            ++counter;
        }

        return colors;
    }

    public void waitUntilStopped(StopperIF stopper) throws InterruptedException {
        stopper.start();

        while(stopper.keepGoing(0)){
            Thread.sleep(1);
        }

        stopper.finish();
    }

//    public void jewel(double initialHeading, boolean isRedSide) throws InterruptedException {
//        _servoArm.setPosition(1);

//        sleep(1250);
//
//        SaturationStopper saturationStopper = new SaturationStopper(_sensorRGBArm, .5f, null);
//        turn(new IMUTurner(5, 1, _imu1, .25, saturationStopper), true, true);
//
//        if(saturationStopper.getSaturation() >= .5){
//            boolean isRed = true;
//
//            if (saturationStopper.getHue() <= 260 && saturationStopper.getHue() >= 120) {
//                RobotLog.d("OctobotMain::jewel()::isRed = false");
//                isRed = false;
//            }
//
//            RobotLog.d("OctobotMain::jewel()::isRed: " + isRed);
//
//            double degreesOffset = getCurrentHeading() - initialHeading;
//
//            RobotLog.d("OctobotMain::jewel()::degreesOffset: " + degreesOffset);
//
//            if (isRed ^ isRedSide) {
//                turn(new IMUTurner(10 - degreesOffset, 1, _imu1, 2, null), true, true);
//                _servoArm.setPosition(0);
//                turn(new IMUTurner(-10, 1, _imu1, 2, null), true, true);
//                // The jewel is red and we are red, or the ball is blue and we are blue
//            } else {
//                // We are the same color as the jewel
//                turn(new IMUTurner(-10 - degreesOffset, 1, _imu1, 2, null), true, true);
//                _servoArm.setPosition(0);
//                turn(new IMUTurner(10, 1, _imu1, 2, null), true, true);
//            }
//        }
//
//        _servoArm.setPosition(0);
//        sleep(250);
//
//        turn(new IMUTurner(-(getCurrentHeading() - initialHeading), 1, _imu1, 1, null), true, true);
   // }

    public void deliverBlockCorner(double targetHeading, Servo grabberServo, Servo otherGrabber, boolean isBlueSide) throws InterruptedException {
        double minDistance = isBlueSide ? 40 : 40;

        SharpDistanceSensor irSensor = _irSensorRight;

        RobotLog.d("OctobotAutonomousBlueCorner::minDistance:: " + minDistance);

        drive(new IMUDriver(0, -.45, _imu1, .04, targetHeading, RobotControl.convertInchesStrafe(20),
                new PillarStopper(irSensor, minDistance, Double.MAX_VALUE, null)),
                true, true);

//        drive(new IMUDriver(0, -.6, _imu1, .04, targetHeading, RobotControl.convertInchesStrafe(.5f), null), true, true);

        drive(new IMUDriver(.6, 0, _imu1, .04, isBlueSide ? targetHeading: targetHeading, RobotControl.convertInches(isBlueSide ? 5 : 7), null), true, true);

        grabberServo.setPosition(.75);
        otherGrabber.setPosition(.75);
    }

    public void deliverBlockCornerSide(double targetHeading, Servo grabberServo, Servo otherGrabber, boolean isBlueSide, boolean isSide) throws InterruptedException {
        double minDistance = isBlueSide ? 40 : 40;

        SharpDistanceSensor irSensor = _irSensorRight;

        RobotLog.d("OctobotAutonomousBlueCorner::minDistance:: " + minDistance);

        drive(new IMUDriver(0, isSide ? .45 : -.45, _imu1, .04, targetHeading, RobotControl.convertInchesStrafe(36),
                        new PillarStopper(irSensor, minDistance, Double.MAX_VALUE, null)),
                true, true);

//        drive(new IMUDriver(0, -.6, _imu1, .04, targetHeading, RobotControl.convertInchesStrafe(.5f), null), true, true);

        drive(new IMUDriver(.6, 0, _imu1, .04, isBlueSide ? targetHeading: targetHeading, RobotControl.convertInches(isBlueSide ? 5 : 7), null), true, true);

        grabberServo.setPosition(.75);
        otherGrabber.setPosition(.75);
    }

//    public void deliverBlockSimple(double initialHeading, Servo grabberServo, boolean isRedSide, boolean isCorner) throws InterruptedException {
//        // Open grabber
//
//        if(grabberServo == null) {
//            _servoGrabberBlue.setPosition(1);
//            _servoGrabberRed.setPosition(1);
//        }
//        else {
//            grabberServo.setPosition(1);
//        }
//
//        // Drive back
//
//        drive(new IMUDriver(-.7, 0, _imu1, .04, isCorner ? initialHeading - 90 : initialHeading, RobotControl.convertInches(3), null), true, true);
//
//        // Close grabber
//
//        if(grabberServo == null) {
//            _servoGrabberBlue.setPosition(0);
//            _servoGrabberRed.setPosition(0);
//        }
//        else {
//            grabberServo.setPosition(0);
//        }
//
//        // Lower lift
//
//        MotorRunner liftLower = new MotorRunner(_motorLift, -1, 0, new TimeStopper(500, null));
//        liftLower.startMotor();
//
//        // Drive forward
//
//        drive(new IMUDriver(.7, 0, _imu1, .04, isCorner ? initialHeading - 90 : initialHeading, RobotControl.convertInches(5 + (isRedSide ? 2 : 0)), null), true, true);
//
//        // Open grabbers
//
//        if(grabberServo == null) {
//            _servoGrabberBlue.setPosition(1);
//            _servoGrabberRed.setPosition(1);
//        }
//        else {
//            grabberServo.setPosition(1);
//        }
//
//    }

//    public void deliverBlockSimple2(double initialHeading, Servo grabberServo, boolean isRedSide, boolean isCorner) throws InterruptedException {
//        // Open grabber
//
//        if(grabberServo == null) {
//            _servoGrabberBlue.setPosition(.75);
//            _servoGrabberRed.setPosition(.75);
//        }
//        else {
//            grabberServo.setPosition(1);
//        }

        // Drive back

//        drive(new IMUDriver(-.7, 0, _imu1, .04, isCorner ? initialHeading - 90 : initialHeading, RobotControl.convertInches(4), null), true, true);
//    }

    public void afterBlockCorner(double initialHeading, boolean side) throws InterruptedException {
        drive(new IMUDriver(-.75, 0, _imu1, .04, initialHeading - 90, RobotControl.convertInches(6), null), true, true);

        turn(new IMUTurner(-90, .7, _imu1, 2, null), true, true);
        turn(new IMUTurner(-90, .7, _imu1, 2, null), true, true);
    }

    public void afterBlockSide(double initialHeading, boolean side) throws InterruptedException {
        drive(new IMUDriver(-.35, 0, _imu1, .04, initialHeading, RobotControl.convertInches(6), null), true, true);

        turn(new IMUTurner(-90, .6, _imu1, 2, null), true, true);
        turn(new IMUTurner(-90, .6, _imu1, 2, null), true, true);
    }


    void grabCubeFromPile(double targetHeading, boolean isRightColumn) throws InterruptedException {
        CubeGrabberStopper cubeGrabberStopper = new CubeGrabberStopper(
                isRightColumn ? _irSensorRight : _irSensorLeft,
                isRightColumn ? _irSensorLeft : _irSensorRight,
                null, true, 30);

        drive(new IMUDriver(0, (isRightColumn ? -1 : 1) * .4f, _imu1, .04, targetHeading, RobotControl.convertInchesStrafe(20), cubeGrabberStopper), true, true);

        double newHeading = getCurrentHeading();

        drive(new IMUDriver(0, (isRightColumn ? -1 : 1) * .4f, _imu1, .04, newHeading, RobotControl.convertInchesStrafe(.5f), null), true, true);

        drive(new IMUDriver(.5, 0, _imu1, .04, newHeading, RobotControl.convertInches(8), null), true, true);

//        _servoGrabberBlue.setPosition(0);
//        _servoGrabberRed.setPosition(0);
//        sleep(750);

        // Raise the lift to get cubes off the floor

        MotorRunner liftRaiser = new MotorRunner(_motorLift, 1, 0, new TimeStopper(900, null));
        liftRaiser.startMotor();

        // Drive backward

        drive(new IMUDriver(-1, 0, _imu1, .04, targetHeading, RobotControl.convertInches(10), null), true, true);

        RobotLog.d("OctobotMain::grabCubeFromPile()::cubeGrabberStopper.getLastPosition(): " + cubeGrabberStopper.getLastPosition());

        // Strafe to correct position

        double strafeAdjustment = (isRightColumn ? -1 : 1 )* (-cubeGrabberStopper.getLastPosition());

        drive(new IMUDriver(0, (strafeAdjustment > 0 ? 1 : -1) * 1f, _imu1, .04, targetHeading, (int)(Math.abs(strafeAdjustment) * 1.25), null), true, true);
    }

    void grabCubeFromPileIR(double targetHeading, boolean isRightColumn, Servo otherGrabber, RelicRecoveryVuMark vuMark) throws InterruptedException {
        CubeGrabberStopper cubeGrabberStopper = new CubeGrabberStopper(
                isRightColumn ? _irSensorRight : _irSensorLeft,
                isRightColumn ? _irSensorLeft : _irSensorRight,
                null, false, 30);

        drive(new IMUDriver(0, .3f, _imu1, .04, targetHeading, RobotControl.convertInchesStrafe(25), cubeGrabberStopper), true, true);

        double newHeading = getCurrentHeading();

        int strafeOffset = 0;

        if(cubeGrabberStopper.isEdgeDetectedLeading()) {
            RobotLog.d("OctobotMain::grabCubeFromPileIR()::Found cube with leading sensor");
//            strafeOffset = -RobotControl.convertInchesStrafe(1f);
            drive(new IMUDriver(0, .4f, _imu1, .04, newHeading, RobotControl.convertInchesStrafe(1f), null), true, true);
        }
        else if(cubeGrabberStopper.isEdgeDetectedTrailing()) {
            RobotLog.d("OctobotMain::grabCubeFromPileIR()::Found cube with trailing sensor");
//            strafeOffset = RobotControl.convertInchesStrafe(1f);
            drive(new IMUDriver(0, -.4f, _imu1, .04, newHeading, RobotControl.convertInchesStrafe(1f), null), true, true);
        }

        drive(new IMUDriver(.5, 0, _imu1, .04, newHeading, RobotControl.convertInches(11), null), true, true);

        otherGrabber.setPosition(0);
        sleep(1000);

        // Raise the lift to get cubes off the floor

        MotorRunner liftRaiser = new MotorRunner(_motorLift, 1, 0, new TimeStopper(900, new PositionStopper(Integer.MIN_VALUE, 2250, null)));
        liftRaiser.startMotor();

        // Drive backward

        drive(new IMUDriver(-1, 0, _imu1, .04, targetHeading, RobotControl.convertInches(13), null), true, true);

        RobotLog.d("OctobotMain::grabCubeFromPileIR()::cubeGrabberStopper.getLastPosition(): " + cubeGrabberStopper.getLastPosition());

        // Strafe to correct position

        int lastPosition = cubeGrabberStopper.getLastPosition();

        RobotLog.d("OctobotMain::grabCubeFromPileIR()::lastPosition: " + lastPosition + " " + RobotControl.convertInchesStrafeInv(lastPosition));

        double strafeAdjustment = -lastPosition + strafeOffset - RobotControl.convertInchesStrafe(5);

        if(vuMark == RelicRecoveryVuMark.LEFT){
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER){
            strafeAdjustment += RobotControl.convertInchesStrafe(7.5f);
        }
        else{
            strafeAdjustment += RobotControl.convertInchesStrafe(15f);
        }

        drive(new IMUDriver(0, (strafeAdjustment > 0 ? 1 : -1) * .3f, _imu1, .04, targetHeading, (int)(Math.abs(strafeAdjustment)), null), true, true);
    }

    void doTheWholeThing(double initialHeading, Servo grabberServo, Servo otherGrabber, RelicRecoveryVuMark vuMark, boolean isBlueSide) throws InterruptedException {
        // Turn toward crypto box

        turn(new IMUTurner(-(getCurrentHeading() - initialHeading) - 90, .6, _imu1, 1, null), true, true);

        deliverBlockCorner(initialHeading - 90, grabberServo, otherGrabber, isBlueSide);

        // Drive backward

        drive(new IMUDriver(-.8, 0, _imu1, .04, initialHeading - 90, RobotControl.convertInches(isBlueSide ? 17 : 15), null), true, true);

        //if blue, true; if red, false

//        _servoGrabberBlue.setPosition(1);
//        _servoGrabberRed.setPosition(1);

        MotorRunner liftLower = new MotorRunner(_motorLift, -1, 0, new TimeStopper(900, null));
        liftLower.startMotor();

        // Turn toward cubes

        RobotLog.d("OctobotAutonomousBlueCorner::Turn toward cubes");

        turn(new IMUTurner(-(getCurrentHeading() - initialHeading) + 89.9, .8, _imu1, 1, null), true, true);

        RobotLog.d("OctobotAutonomousBlueCorner::Grab from pile");

        grabCubeFromPile(initialHeading + 90, vuMark == RelicRecoveryVuMark.RIGHT);

        // Turn toward crypto box

        MotorRunner liftRaiser = new MotorRunner(_motorLift, 1, 0, new TimeStopper(1500, null));
        liftRaiser.startMotor();

        turn(new IMUTurner(-(getCurrentHeading() - initialHeading) - 90, .7, _imu1, 1, null), true, true);

        // Drive to crypto box

        drive(new IMUDriver(.7, 0, _imu1, .04, initialHeading - 90, RobotControl.convertInches(isBlueSide ? 10 : 15), null), true, true);

//        deliverBlockSimple2(initialHeading, null, true, true);
    }

    void doTheWholeThing2(double initialHeading, Servo grabberServo, Servo otherGrabber, RelicRecoveryVuMark vuMark, boolean isBlueSide) throws InterruptedException {
        // Turn toward crypto box

        RobotLog.d("OctobotMain::doTheWholeThing2::Turn right 90");

        turn(new IMUTurner(-(getCurrentHeading() - initialHeading) + 90, .6, _imu1, 1, null), true, true);

        RobotLog.d("OctobotMain::doTheWholeThing2::Start spinner");

        RobotLog.d("OctobotMain::doTheWholeThing2::Drive toward glyphs");

        drive(new IMUDriver(.8, 0, _imu1, .04, initialHeading + 90, RobotControl.convertInches(3), null), true, true);

        RobotLog.d("OctobotMain::doTheWholeThing2::Open other grabber");

        grabberServo.setPosition(1);

        RobotLog.d("OctobotMain::doTheWholeThing2::Lower lift");

        MotorRunner liftLower = new MotorRunner(_motorLift, -1, 0, new TimeStopper(800, new PositionStopper(120, Integer.MAX_VALUE, null)));
        liftLower.startMotor();

        /*
            Make sure that the spinner has stopped
         */

        RobotLog.d("OctobotMain::doTheWholeThing2::Make sure spinner has stopped");

        RobotLog.d("OctobotMain::doTheWholeThing2::Grab cube from pile");

        grabCubeFromPileIR(initialHeading + 90, vuMark == RelicRecoveryVuMark.RIGHT, grabberServo, vuMark);

        // Turn toward crypto box

        RobotLog.d("OctobotMain::doTheWholeThing2::Turn toward crypto box");

        turn(new IMUTurner(90, .7, _imu1, 5, null), true, true);
        turn(new IMUTurner(90, .7, _imu1, 5, null), true, true);
        turn(new IMUTurner(-(getCurrentHeading() - initialHeading) - 90, .7, _imu1, 1, null), true, true);

        // Drive to crypto box

        RobotLog.d("OctobotMain::doTheWholeThing2::Drive to crypto box");

        //drive(new IMUDriver(.7, 0, _imu1, .04, initialHeading - 90, RobotControl.convertInches(3), null), true, true);

        RobotLog.d("OctobotMain::doTheWholeThing2::Deliver block");

        deliverBlockCorner(initialHeading - 90, grabberServo, otherGrabber, isBlueSide);

        RobotLog.d("OctobotMain::doTheWholeThing2::Drive back");

        drive(new IMUDriver(-.6, 0, _imu1, .04, initialHeading - 90, RobotControl.convertInches(5), null), true, true);
    }

}

