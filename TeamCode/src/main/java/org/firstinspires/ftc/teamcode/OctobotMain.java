package org.firstinspires.ftc.teamcode;

import com.google.gson.Gson;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

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


    public DigitalChannel _button0 = null;
    public DigitalChannel _button1 = null;
    public DigitalChannel _button2 = null;
    public DigitalChannel _button3 = null;

    BNO055IMU _imu;
    BNO055IMU _imu1;
    BNO055IMU _imu2;


    //DeviceInterfaceModule _cdim;
    TCS34725_ColorSensor _sensorRGB;

    public Servo _servoTop = null;
    public Servo _servoBottom = null;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int RGB_LED_CHANNEL = 5;

    VuforiaLocalizer _vuforia;
    VuforiaTrackables _beacons;

    static final int BLUE_NEAR = 0;
    static final int RED_FAR = 1;
    static final int BLUE_FAR = 2;
    static final int RED_NEAR = 3;

//    class CalibrationData {
//        double _analog0Min = 0;
//        double _analog0Max = .132;
//
//        double _analog1Min = 0.0587;
//        double _analog1Max = 0.2004;
//
//        CalibrationData() {
//        }
//
//        public String serialize() {
//            return new Gson().toJson(this);
//        }
//    }
//
//    CalibrationData _calibrationData = new CalibrationData();

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



//        RobotControl.resetAllDriveMotorEncoders(_motorA, _motorB, _motorC, _motorD, this);

        RobotLog.d("OctobotMain::initialize::_motorA::" + _motorA);
        RobotLog.d("OctobotMain::initialize::_motorB::" + _motorB);
        RobotLog.d("OctobotMain::initialize::_motorC::" + _motorC);
        RobotLog.d("OctobotMain::initialize::_motorD::" + _motorD);
        RobotLog.d("OctobotMain::initialize::_motorSlide::" + _motorSlide);
        RobotLog.d("OctobotMain::initialize::_motorLift::" + _motorLift);
        RobotLog.d("OctobotMain::initialize::_motorSpinner::" + _motorSpinner);
//        RobotLog.d("OctobotMain::initialize::_motorScooper::" + _motorScooper);

        /*
           Initialize LED's
         */

//        _led0 = hardwareMap.digitalChannel.get("led_0");
//        _led0.setMode(DigitalChannelController.Mode.OUTPUT);
//
//        _led1 = hardwareMap.digitalChannel.get("led_1");
//        _led1.setMode(DigitalChannelController.Mode.OUTPUT);
//
//        _led0.setState(false);
//        _led1.setState(false);

        /*
            Initialize light sensors
         */
//
//        _analog0 = hardwareMap.analogInput.get("analog_0");
//        _analog1 = hardwareMap.analogInput.get("analog_1");

        /*
            Initialize pusher button
         */

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

        _servoTop = hardwareMap.servo.get("servo_top");
        _servoBottom = hardwareMap.servo.get("servo_bottom");
//

//
//        _servoShooter = hardwareMap.servo.get("servo_shooter");
//        _servoShooter.setPosition(0);

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
        _imu = hardwareMap.get(BNO055IMU.class, "imu");
        _imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
        _imu2 = hardwareMap.get(BNO055IMU.class, "imu2");

        _imu.initialize(parameters);
        _imu1.initialize(parameters);
        _imu2.initialize(parameters);

        /*
            Initialize RGB sensor
        */

        // get a reference to our DeviceInterfaceModule object.
//        _cdim = hardwareMap.deviceInterfaceModule.get("dim");
//
//        // set the digital channel to output mode.
//        // remember, the Adafruit sensor is actually two devices.
//        // It's an I2C sensor and it's also an LED that can be turned on or off.
//        _cdim.setDigitalChannelMode(RGB_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
//
//        // get a reference to our ColorSensor object.
//        _sensorRGB = new TCS34725_ColorSensor(hardwareMap, "color");
//
//        // turn the LED off in the beginning, just so user will know that the sensor is active.
//        _cdim.setDigitalChannelState(RGB_LED_CHANNEL, false);
    }

//    void loadCalibration() {
        /*
            Load calibration data
         */

//        _calibrationData = new Gson().fromJson("OctobotMainCalibration", CalibrationData.class);

//        RobotLog.d("OctobotMain::loadCalibration::analog0 %.4f %.4f", _calibrationData._analog0Min, _calibrationData._analog0Max);
//        RobotLog.d("OctobotMain::loadCalibration::analog1 %.4f %.4f", _calibrationData._analog1Min, _calibrationData._analog1Max);
//    }

//    void saveCalibration() {
//        String filename = "AdafruitIMUCalibration.json";
//        File file = AppUtil.getInstance().getSettingsFile(filename);
//        ReadWriteFile.writeFile(file, _calibrationData.serialize());
//    }
//
//    void calibrate() throws InterruptedException {
//        //  Calibrate the light sensor
//
//        calibrateLight();
//    }

//    public void calibrateLight() throws InterruptedException {
//        _led0.setState(true);
//        _led1.setState(true);

//        _calibrationData._analog0Min = Float.MAX_VALUE;
//        _calibrationData._analog0Max = -Float.MAX_VALUE;
//
//        _calibrationData._analog1Min = Float.MAX_VALUE;
//        _calibrationData._analog1Max = -Float.MAX_VALUE;
//
//        telemetry.addData(">", "Calibrating Light Sensors");
//        telemetry.update();
//
//        long startTime = System.currentTimeMillis();
//
//        while (System.currentTimeMillis() - startTime < 15000) {
////            double voltage0 = _analog0.getVoltage();
////            double voltage1 = _analog1.getVoltage();
//
////            _calibrationData._analog0Min = Math.min(_calibrationData._analog0Min, voltage0);
////            _calibrationData._analog0Max = Math.max(_calibrationData._analog0Max, voltage0);
////
////            _calibrationData._analog1Min = Math.min(_calibrationData._analog1Min, voltage1);
////            _calibrationData._analog1Max = Math.max(_calibrationData._analog1Max, voltage1);
//
//            telemetry.addData("Time left", String.format("%d", (int)(15 - (System.currentTimeMillis() - startTime ) / 1000)));
//            telemetry.addData("analog0 Range", String.format("%.2f %.2f",_calibrationData._analog0Min, _calibrationData._analog0Max));
//            telemetry.addData("analog1 Range", String.format("%.2f %.2f",_calibrationData._analog1Min, _calibrationData._analog1Max));
//            telemetry.update();
//
//            RobotLog.d("calibrateLight::analog0 %.4f %.4f  analog1 %.4f %.4f",_calibrationData._analog0Min, _calibrationData._analog0Max, _calibrationData._analog1Min, _calibrationData._analog1Max);
//
//            Thread.sleep(10);
//        }
//
//        telemetry.addData(">", "Calibration complete");
//        telemetry.update();
//
//        Thread.sleep(1000);
//
//        /*
//            Bad calibration (or someone forgot), do something reasonable
//         */
//
//        if(_calibrationData._analog0Max - _calibrationData._analog0Min < .05) {
//            _calibrationData._analog0Max = _calibrationData._analog0Min + (0.1613 - 0.0293);
//        }
//
//        if(_calibrationData._analog1Max - _calibrationData._analog1Min < .05) {
//            _calibrationData._analog1Max = _calibrationData._analog1Min + (0.2248 - 0.0831);
//        }
//
////        _led0.setState(false);
////        _led1.setState(false);
//    }
//
//    double getTarget0() {
//        return (_calibrationData._analog0Min + _calibrationData._analog0Max) / 2;
//    }
//
//    double getTarget1() {
//        return (_calibrationData._analog1Min + _calibrationData._analog1Max) / 2;
//    }

//    public void calibrateGyro() throws InterruptedException {
//        _gyro.calibrate();
//        _gyro.resetZAxisIntegrator();
//
//        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
//        telemetry.update();
//
//        while(_gyro.isCalibrating()) {
//            Thread.sleep(200);
//            idle();
//        }
//
//        telemetry.addData("Relative Heading", _gyro.getIntegratedZValue());
//        telemetry.update();
//    }

    double getCurrentHeading() {
        Orientation angles = _imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /*
        Iintialize Vuforia library, beacons and camera position
     */

    void initializeVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQMmXDH/////AAAAGXvAv3V7xURDoUrFi6n/qG0LC5kmnnRDMaBE2DbHd3pUuUoMWpX4gA0ZWWxCDoU5Fd189JV3/q1hYeTJGgSschiTneHnBGjCp3KK+ADOnwyxeu8odM4s/CFViozG+D8yyt6sORwv3yWK4g05DOP0dMpUdkUVuADqRUoIYPex/dkJajOXnhteO66gPxZOMCApNuTue1pyYueBeOIGWYC2DY7XQP2DeEL1BssxiXupkFtOsQWE6FWeKvKRwKdTmDQymM+s5SRfhxF/8Yd85keIe0jL/OyzeSIpq8RnHOGz2HjTEQK9UxWD/+A8jy584gdb8X3613zXPURYGKhU657+asjXTb1Yg8smo8CRAUcpEE24";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.cameraMonitorFeedback = AXES; //set camera feedback icon (An axis is placed on photo in preview)
        parameters.useExtendedTracking = false;

        _vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        _beacons = _vuforia.loadTrackablesFromAsset("FTC_2016-17"); //load beacon images

        VuforiaTrackable blueNearTarget = _beacons.get(BLUE_NEAR); // Wheels
        blueNearTarget.setName("Blue Near");

        VuforiaTrackable blueFarTarget = _beacons.get(BLUE_FAR); // Legos
        blueFarTarget.setName("Blue Far");

        VuforiaTrackable redNearTarget = _beacons.get(RED_NEAR); // Gears
        redNearTarget.setName("Red Near");

        VuforiaTrackable redFarTarget = _beacons.get(RED_FAR); // Tools
        redFarTarget.setName("Red Far");

        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float mmFTCTileWidth = 24 * mmPerInch;
        float mmFTCTargetHeight = (1.5f + 8.5f / 2) * mmPerInch;
        float mmCameraHeight = 9f * mmPerInch;

        // Blue Near location

        OpenGLMatrix blueNearLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                .translation(mmFTCTileWidth / 2, mmFTCFieldWidth / 2, mmFTCTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueNearTarget.setLocation(blueNearLocationOnField);
        RobotLog.ii("OctobotMain::initializeVuforia", "Blue Near=%s", VuforiaUtil.formatOpenGLMatrix(blueNearLocationOnField));

        // Blue Far location

        OpenGLMatrix blueFarLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                .translation(-mmFTCTileWidth * 3 / 2, mmFTCFieldWidth / 2, mmFTCTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueFarTarget.setLocation(blueFarLocationOnField);
        RobotLog.ii("OctobotMain::initializeVuforia", "Blue Far=%s", VuforiaUtil.formatOpenGLMatrix(blueFarLocationOnField));

        // Red Near location

        OpenGLMatrix redNearLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 2, -mmFTCTileWidth / 2, mmFTCTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redNearTarget.setLocation(redNearLocationOnField);
        RobotLog.ii("OctobotMain::initializeVuforia", "Red Near=%s", VuforiaUtil.formatOpenGLMatrix(redNearLocationOnField));

        // Red Far location

        OpenGLMatrix redFarLocationOnField = OpenGLMatrix
                    /* Then we translate the target off to the RED WALL. Our translation here
                    is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 2, mmFTCTileWidth * 3 / 2, mmFTCTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                            /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redFarTarget.setLocation(redFarLocationOnField);
        RobotLog.ii("OctobotMain::initializeVuforia", "Red Far=%s", VuforiaUtil.formatOpenGLMatrix(redFarLocationOnField));

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */

            /* .translation(0,0,mmBotWidth / 2)  80 1200 278 */
            /* .translation(0,0,-mmBotWidth / 2) 538 1200 274 */
            /* .translation(0,mmBotWidth / 2,0)  310 1220 502 */
            /* .translation(0,-mmBotWidth / 2,0)  310 1182 48 */
            /* .translation(mmBotWidth / 2,0,0)  310 1430 255 */
            /* .translation(-mmBotWidth / 2,0,0)  309 974 296 */

        /*
            -90, -90, 0 appeared to work for absolution position, but -90,90,0 works better for position relative to beacon
         */

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(-mmBotWidth / 2, -mmCameraHeight, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES, -90, 90, 0));
        RobotLog.ii("OctobotMain::initializeVuforia", "Phone = %s", VuforiaUtil.formatOpenGLMatrix(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */

        for (VuforiaTrackable trackable : _beacons) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    }

    void startVuforia() {
        _beacons.activate();
    }

    void stopVuforia() {
        _beacons.deactivate();
    }

    OpenGLMatrix getVuforiaLocation() {
        OpenGLMatrix lastLocation = null;

        for (VuforiaTrackable trackable : _beacons) {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        return lastLocation;
    }

    VuforiaTrackableDefaultListener getBeaconListener(int beaconIndex) {
        return (VuforiaTrackableDefaultListener) _beacons.get(beaconIndex).getListener();
    }

    boolean isBeaconVisible(VuforiaTrackableDefaultListener listener) throws InterruptedException {
        boolean isVisible = false;

        int count = 0;

        while(!isVisible && count < 100) {
            isVisible =  (listener.getRawPose() != null);
            ++count;
            Thread.sleep(10);
        }

        return isVisible;
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

    public void drive(DriverIF driver) throws InterruptedException {
        driver.start();

        resetAllDriveMotorEncoders();
        runUsingEncoders();

        boolean keepGoing = true;

        DriverIF.Steerage lastSteerage = null;

        long startTime = System.currentTimeMillis();
        int numSteeringUpdates = 0;

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

                RobotLog.d("OctobotMain::drive::frontRight: " + frontRight);
                RobotLog.d("OctobotMain::drive::rearRight: " + rearRight);
                RobotLog.d("OctobotMain::drive::frontLeft: " + frontLeft);
                RobotLog.d("OctobotMain::drive::rearLeft: " + rearLeft);
            }

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            int position = (Math.abs(positionA) + Math.abs(positionB) + Math.abs(positionC) + Math.abs(positionD)) / 4;

            keepGoing = driver.keepGoing(position);

            Thread.sleep(5);

            idle();
        }

        RobotLog.d("OctobotMain::drive::steering update interval: " + (System.currentTimeMillis() - startTime) / numSteeringUpdates);

        stopMotors();

        driver.finish();
    }

    public void turn(TurnerIF turner) throws InterruptedException {
        turner.start();

        resetAllDriveMotorEncoders();
        runUsingEncoders();

        boolean keepGoing = true;

        double lastScaleFactor = 0;
        double lastPower = 0;

        while(keepGoing) {
            idle();

            double power = turner.getPower();
            double scaleFactor = turner.getScaleFactor();

            if(power != lastPower || scaleFactor != lastScaleFactor) {
                RobotLog.d("OctobotMain::turn::power: " + power);
                RobotLog.d("OctobotMain::turn::scaleFactor: " + scaleFactor);

                if (Double.isNaN(scaleFactor)) {
                    keepGoing = false;
                } else {
                    _motorA.setPower(-power * scaleFactor);
                    _motorB.setPower(-power * scaleFactor);
                    _motorC.setPower(power * scaleFactor);
                    _motorD.setPower(power * scaleFactor);
                }

                lastPower = power;
                lastScaleFactor = scaleFactor;
            }
        }

        stopMotors();
    }
}

