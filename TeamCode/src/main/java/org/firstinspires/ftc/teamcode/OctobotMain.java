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
public abstract class OctobotMain extends LinearOpMode {
    protected DcMotor _motorA;
    protected DcMotor _motorB;
    protected DcMotor _motorC;
    protected DcMotor _motorD;
    protected DcMotor _motorIntake;
    protected DcMotor _motorLift;
    protected DcMotor _motorLift2;


    public DigitalChannel _frontSliderLimit = null;
    public DigitalChannel _rearSliderLimit = null;
    public DigitalChannel _button2 = null;
    public DigitalChannel _button3 = null;

     BNO055IMU _imu1;

    ColorSensor _sensorRGB;
    NormalizedColorSensor _sensorRGBArm;

    public Servo _servoA = null;

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

    /**
     * Initialize all sensors, motors, etc.
     *
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

        _motorIntake = hardwareMap.dcMotor.get("motor_intake");
        _motorIntake.setDirection(DcMotor.Direction.REVERSE);
        _motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorLift = hardwareMap.dcMotor.get("motor_lift");
        _motorLift.setDirection(DcMotor.Direction.REVERSE);
        _motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _motorLift2 = hardwareMap.dcMotor.get("motor_lift2");
        _motorLift2.setDirection(DcMotor.Direction.REVERSE);
        _motorLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RobotLog.d("OctobotMain::initialize::_motorA::" + _motorA);
        RobotLog.d("OctobotMain::initialize::_motorB::" + _motorB);
        RobotLog.d("OctobotMain::initialize::_motorC::" + _motorC);
        RobotLog.d("OctobotMain::initialize::_motorD::" + _motorD);
        RobotLog.d("OctobotMain::initialize::_motorIntake::" + _motorIntake);
        RobotLog.d("OctobotMain::initialize::_motorLift::" + _motorLift);
        RobotLog.d("OctobotMain::initialize::_motorLift2::" + _motorLift2);

        /*
            Initialize pusher button
         */

        RobotLog.d("OctobotMain::initialize::initialize buttons");

//        _frontSliderLimit = hardwareMap.digitalChannel.get("button_0");
//        _frontSliderLimit.setMode(DigitalChannel.Mode.INPUT);
//
//        _rearSliderLimit = hardwareMap.digitalChannel.get("button_1");
//        _rearSliderLimit.setMode(DigitalChannel.Mode.INPUT);
//
//        _button2 = hardwareMap.digitalChannel.get("button_2");
//        _button2.setMode(DigitalChannel.Mode.INPUT);
//
//        _button3 = hardwareMap.digitalChannel.get("button_3");
//        _button3.setMode(DigitalChannel.Mode.INPUT);

        /*
            Initialize servos
         */

        RobotLog.d("OctobotMain::initialize::initialize servos");

        _servoA = hardwareMap.servo.get("servo_box");

        /*
            Initialize IMU
         */

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = false;
//        parameters.loggingTag = "IMU";
   //     parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        // _imu1 = hardwareMap.get(BNO055IMU.class, "imu1");

        //_imu1.initialize(parameters);

        RobotLog.d("OctobotMain::initialize::end");
    }

    double getCurrentHeading() {
        Orientation angles = _imu1.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        return angles.firstAngle;
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
        driver.start();

        resetAllDriveMotorEncoders();

        if (runUsingEncoders) {
            runUsingEncoders();
        } else {
            runWithoutEncoders();
        }

        boolean keepGoing = true;

        DriverIF.Steerage lastSteerage = null;

        long startTime = System.currentTimeMillis();
        int numSteeringUpdates = 0;

        int lastPositionA = _motorA.getCurrentPosition();
        int lastPositionB = _motorB.getCurrentPosition();
        int lastPositionC = _motorC.getCurrentPosition();
        int lastPositionD = _motorD.getCurrentPosition();

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

            if (lastSteerage == null || !steerage.equals(lastSteerage)) {
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
            }

            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            /*
                If the change in encoder position is opposite what should have happened based on the power
                applied then don't use that encoder since it may be bad
             */

            if (lastPowerA * (positionA - lastPositionA) < 0) {
                keepPositionA = 0;
            }

            if (lastPowerB * (positionB - lastPositionB) < 0) {
                keepPositionB = 0;
            }

            if (lastPowerC * (positionC - lastPositionC) < 0) {
                keepPositionC = 0;
            }

            if (lastPowerD * (positionD - lastPositionD) < 0) {
                keepPositionD = 0;
            }

            lastPositionA = positionA;
            lastPositionB = positionB;
            lastPositionC = positionC;
            lastPositionD = positionD;

            lastPowerA = _motorA.getPower();
            lastPowerB = _motorB.getPower();
            lastPowerC = _motorC.getPower();
            lastPowerD = _motorD.getPower();

            if (keepPositionA + keepPositionB + keepPositionC + keepPositionD != 0) {
                int position = (Math.abs(positionA) * keepPositionA + Math.abs(positionB) * keepPositionB + Math.abs(positionC) * keepPositionC + Math.abs(positionD) * keepPositionD) /
                        (keepPositionA + keepPositionB + keepPositionC + keepPositionD);

                keepGoing = driver.keepGoing(position);
            } else {
                keepGoing = false;
            }

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

    public void waitUntilStopped(StopperIF stopper) throws InterruptedException {
        stopper.start();

        while (stopper.keepGoing(0)) {
            Thread.sleep(1);
        }

        stopper.finish();
    }
}