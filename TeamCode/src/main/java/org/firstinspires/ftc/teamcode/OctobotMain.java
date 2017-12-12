package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;



import java.io.File;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.util.Arrays;

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


    public DigitalChannel _button0 = null;
    public DigitalChannel _button1 = null;
    public DigitalChannel _button2 = null;
    public DigitalChannel _button3 = null;

    BNO055IMU _imu;
    BNO055IMU _imu1;
    BNO055IMU _imu2;


    //DeviceInterfaceModule _cdim;
    NormalizedColorSensor _sensorRGB;
    NormalizedColorSensor _sensorRGBArm;

    public Servo _servoTop = null;
    public Servo _servoBottom = null;
    public Servo _servoArm = null;
    public Servo _servoRelic = null;
    public Servo _servoLock = null;

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
        IR Sensors
     */

    SharpDistanceSensor _irSensorLeft;
    SharpDistanceSensor _irSensorRight;

    /*
        Sonar sensors
     */

    public MaxSonarI2CXL _sonarLeft;
    public MaxSonarI2CXL _sonarRight;
    public SonarArrayManager _sonarArrayManager;

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

        _button0 = hardwareMap.digitalChannel.get("button_0");
        _button0.setMode(DigitalChannel.Mode.INPUT);

        _button1 = hardwareMap.digitalChannel.get("button_1");
        _button1.setMode(DigitalChannel.Mode.INPUT);

        _button2 = hardwareMap.digitalChannel.get("button_2");
        _button2.setMode(DigitalChannel.Mode.INPUT);

        _button3 = hardwareMap.digitalChannel.get("button_3");
        _button3.setMode(DigitalChannel.Mode.INPUT);

        /*
            Initialize servos
         */

        RobotLog.d("OctobotMain::initialize::initialize servos");

        _servoTop = hardwareMap.servo.get("servo_top");
        _servoBottom = hardwareMap.servo.get("servo_bottom");
        _servoArm = hardwareMap.servo.get("servo_arm");
        _servoRelic = hardwareMap.servo.get("servo_relic");
        _servoLock = hardwareMap.servo.get("servo_lock");

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

        _sensorRGB = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
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

        _sonarLeft = hardwareMap.get(MaxSonarI2CXL.class, "sonar_left");
        _sonarRight = hardwareMap.get(MaxSonarI2CXL.class, "sonar_right");

        _sonarLeft.setI2cAddress(I2cAddr.create8bit(0xE0));
        _sonarRight.setI2cAddress(I2cAddr.create8bit(0xDE));

        _sonarArrayManager = new SonarArrayManager();
        _sonarArrayManager.addSonar("left", _sonarLeft);
        _sonarArrayManager.addSonar("right", _sonarRight);

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


    public void drive(DriverIF driver) throws InterruptedException {
        RobotLog.d("OctobotMain::drive()::A");
        driver.start();

        RobotLog.d("OctobotMain::drive()::B");

        resetAllDriveMotorEncoders();
        runUsingEncoders();

        RobotLog.d("OctobotMain::drive()::C");

        boolean keepGoing = true;

        DriverIF.Steerage lastSteerage = null;

        long startTime = System.currentTimeMillis();
        int numSteeringUpdates = 0;

        RobotLog.d("OctobotMain::drive()::D");

        _sonarRight.startAutoPing(100);
        _sonarLeft.startAutoPing(100);

        while (keepGoing) {
            RobotLog.d("OctobotMain::drive()::E");

            DriverIF.Steerage steerage = driver.getSteerage();

            if(lastSteerage == null || !steerage.equals(lastSteerage)) {
                RobotLog.d("OctobotMain::drive()::F");

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

                RobotLog.d("OctobotMain::drive()::");

                _motorA.setPower(frontRight);
                _motorB.setPower(rearRight);
                _motorC.setPower(frontLeft);
                _motorD.setPower(rearLeft);

                RobotLog.d("OctobotMain::drive()::frontRight: " + frontRight);
                RobotLog.d("OctobotMain::drive()::rearRight: " + rearRight);
                RobotLog.d("OctobotMain::drive()::frontLeft: " + frontLeft);
                RobotLog.d("OctobotMain::drive()::rearLeft: " + rearLeft);
            }

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            int position = (Math.abs(positionA) + Math.abs(positionB) + Math.abs(positionC) + Math.abs(positionD)) / 4;

            RobotLog.d("OctobotMain::drive()::G");

            keepGoing = driver.keepGoing(position);

            RobotLog.d("OctobotMain::drive()::keepGoing: " + keepGoing);

            Thread.sleep(5);

            RobotLog.d("OctobotMain::drive::H");

            idle();
        }

        _sonarRight.stopAutoPing();
        _sonarLeft.stopAutoPing();

        RobotLog.d("OctobotMain::drive::steering update interval: " + (System.currentTimeMillis() - startTime) / numSteeringUpdates);

        stopMotors();

        RobotLog.d("OctobotMain::drive()::I");

        driver.finish();

        RobotLog.d("OctobotMain::drive()::J");

    }

    public void turn(TurnerIF turner) throws InterruptedException {

        RobotLog.d("OctobotMain::turn()::D");

        turner.start();

        runUsingEncoders();

        RobotLog.d("OctobotMain::turn()::E");

        boolean keepGoing = true;

        double lastScaleFactor = 0;
        double lastPower = 0;

        while(keepGoing) {
            RobotLog.d("OctobotMain::turn()::F");

//            waitForNextHardwareCycle();

            RobotLog.d("OctobotMain::turn()::G");

            double power = turner.getPower();

            RobotLog.d("OctobotMain::turn()::H");

            double scaleFactor = turner.getScaleFactor();

            RobotLog.d("OctobotMain::turn()::power: " + power);
            RobotLog.d("OctobotMain::turn()::scaleFactor: " + scaleFactor);

            if(power != lastPower || scaleFactor != lastScaleFactor) {
                RobotLog.d("OctobotMain::turn()::I");

                if (Double.isNaN(scaleFactor)) {
                    keepGoing = false;
                } else {
                    RobotLog.d("OctobotMain::turn()::J");

                    _motorA.setPower(-power * scaleFactor);
                    _motorB.setPower(-power * scaleFactor);
                    _motorC.setPower(power * scaleFactor);
                    _motorD.setPower(power * scaleFactor);
                }

                lastPower = power;
                lastScaleFactor = scaleFactor;
            }

            RobotLog.d("OctobotMain::turn()::K");

            Thread.sleep(5);

//            idle();
//            try {
//                Thread.yield();
//            }
//            catch(Throwable t) {
//            }
        }

        RobotLog.d("OctobotMain::turn()::L");

        stopMotors();

        RobotLog.d("OctobotMain::turn()::M");
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

        RobotLog.d("Motor Power " + motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);
        telemetry.addData("Motor Power", motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);

        _motorA.setPower(motorAPower);
        _motorB.setPower(motorBPower);
        _motorC.setPower(motorCPower);
        _motorD.setPower(motorDPower);
    }

    public Servo getGrabberServo(){
        float[] hsvValuesGrabber = new float[3];

        NormalizedRGBA colorsGrabber = _sensorRGB.getNormalizedColors();

        Color.colorToHSV(colorsGrabber.toColor(), hsvValuesGrabber);

        float hueGrabber = hsvValuesGrabber[0];
        float saturationGrabber = hsvValuesGrabber[1];

        boolean isBlue = true;

        if (hueGrabber >= 0 && hueGrabber <= 90) {
            isBlue = false;
        } else if (hueGrabber <= 260 && hueGrabber >= 120) {
            isBlue = true;
        }

        if(!isBlue){
            return _servoBottom;
        }
        else{
            return _servoTop;
        }
    }
    public RelicRecoveryVuMark getVuforia(){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        while(vuMark == RelicRecoveryVuMark.UNKNOWN){
            vuMark = getDisplayedVuMark();
            sleep(5);
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

    public void jewel(double initialHeading, boolean side) throws InterruptedException {
        _servoArm.setPosition(1);

        sleep(1250);

        float[] hsvValuesArm = new float[3];

        NormalizedRGBA colorsArm = getColorsRobust(_sensorRGBArm);

        Color.colorToHSV(colorsArm.toColor(), hsvValuesArm);

        float hueArm = hsvValuesArm[0];
        float saturationArm = hsvValuesArm[1];

        RobotLog.d("OctobotMain::jewel()::hueArm: " + hueArm);
        RobotLog.d("OctobotMain::jewel()::saturationArm: " + saturationArm);

        if(saturationArm < .5) {
            RobotLog.d("OctobotMain::jewel()::turn closer");
            turn(new IMUTurner(2, .1, _imu1, .2, .25));
            colorsArm = getColorsRobust(_sensorRGBArm);
            Color.colorToHSV(colorsArm.toColor(), hsvValuesArm);
            hueArm = hsvValuesArm[0];
            saturationArm = hsvValuesArm[1];
            RobotLog.d("OctobotMain::jewel()::hueArm: " + hueArm);
            RobotLog.d("OctobotMain::jewel()::saturationArm: " + saturationArm);
        }

        colorsArm = getColorsRobust(_sensorRGBArm);
        Color.colorToHSV(colorsArm.toColor(), hsvValuesArm);
        hueArm = hsvValuesArm[0];
        saturationArm = hsvValuesArm[1];

        if(saturationArm < .5){
            turn(new IMUTurner(-(getCurrentHeading() - initialHeading), .1, _imu1, .2, .25));

            _servoArm.setPosition(0);
        }
        else {
            boolean isRed = true;


            if (hueArm <= 260 && hueArm >= 120) {
                RobotLog.d("OctobotMain::jewel()::isRed = false");
                isRed = false;
            }

            RobotLog.d("OctobotMain::jewel()::isRed: " + isRed);

            if (isRed) {
                turn(new IMUTurner(15 * (side ? 1 : -1), .7, _imu1, .2, 2));
                turn(new IMUTurner(-15 * (side ? 1 : -1), .3, _imu1, .2, 2));
            } else if (!isRed) {
                turn(new IMUTurner(-15 * (side ? 1 : -1), .7, _imu1, .2, 2));
                turn(new IMUTurner(15 * (side ? 1 : -1), .3, _imu1, .2, 2));
            }

            turn(new IMUTurner(-(getCurrentHeading() - initialHeading), .1, _imu1, .2, .25));

            _servoArm.setPosition(0);
        }
    }

    public void deliverBlockCorner(double initialHeading, Servo _servo, boolean side) throws InterruptedException {
         SharpDistanceSensor irSensor = (side ? _irSensorRight : _irSensorLeft);

        Thread.sleep(100);

        drive(new IMUDriver(.35, 0, _imu1, .04, initialHeading - 90, RobotControl.convertInches(side ? 2 : 5), null));

        double minDistance = irSensor.getDistance() - 8;

        RobotLog.d("OctobotAutonomousBlueCorner::minDistance:: " + minDistance);

        drive(new DistanceStopper((side ? _irSensorRight : _irSensorLeft), minDistance, Double.MAX_VALUE,
                new IMUDriver(0, side ? .25 : -.25, _imu1, .04, initialHeading - 90, RobotControl.convertInchesStrafe(8), null)));

        drive(new IMUDriver(0, side ? -.25 : .25, _imu1, .04, initialHeading - 90, RobotControl.convertInchesStrafe(2.5f), null));

        while(!_button1.getState()){
            _motorSlide.setPower(-1);
            Thread.sleep(5);
            idle();
        }

        _motorSlide.setPower(0);

        _servo.setPosition(0);
    }

    public void deliverBlockSide(double initialHeading, Servo _servo, boolean side) throws InterruptedException {
        drive(new IMUDriver(.35, 0, _imu1, .04, initialHeading, RobotControl.convertInches(2), null));

        DistanceSensorIF distanceSensor = (side ? _irSensorRight : _irSensorLeft);

        double minDistance = 20;

        RobotLog.d("OctobotAutonomousBlueCorner::minDistance:: " + minDistance);

        drive(new PillarStopper(distanceSensor, minDistance, Double.MAX_VALUE,
                new IMUDriver(0, side ? .25 : -.25, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(8), null)));

        drive(new IMUDriver(0, side ? -.25 : .25, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(1.5f), null));

        while(!_button1.getState()){
            _motorSlide.setPower(-1);
            Thread.sleep(5);
            idle();
        }

        _motorSlide.setPower(0);

        _servo.setPosition(0);
    }

    public void afterBlockCorner(double initialHeading, boolean side) throws InterruptedException {
        drive(new IMUDriver(-.35, 0, _imu1, .04, initialHeading - 90, RobotControl.convertInches(9), null));

        turn(new IMUTurner(-90, .6, _imu1, .2, 2));
        turn(new IMUTurner(-90, .6, _imu1, .2, 2));

        drive(new IMUDriver(-.35, 0, _imu1, .04, initialHeading + 90, RobotControl.convertInches(13), null));
    }

    public void afterBlockSide(double initialHeading, boolean side) throws InterruptedException {
        drive(new IMUDriver(-.35, 0, _imu1, .04, initialHeading, RobotControl.convertInches(9), null));

        turn(new IMUTurner(-90, .6, _imu1, .2, 2));
        turn(new IMUTurner(-90, .6, _imu1, .2, 2));
    }

    public void raiseLift1500() throws InterruptedException {
        _motorLift.setPower(1);

//        while(_motorLift.getCurrentPosition() < 1500) {
//            Thread.sleep(5);
//            idle();
//        }
        Thread.sleep(1000)   ;

        _motorLift.setPower(0);
    }


}

