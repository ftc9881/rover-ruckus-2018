package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

@TeleOp(name = "Test Op Mode", group = "octobot")
@Disabled
public class TestOpMode extends LinearOpMode {
    /* Public OpMode members. */
    public DcMotor _motorA = null;
    public DcMotor _motorB = null;
    public DcMotor _motorC = null;
    public DcMotor _motorD = null;
    public DcMotor _MotorE = null;
    public DcMotor _MotorF = null;

//    ModernRoboticsI2cGyro _gyro;   // Hardware Device Object

    boolean left;
    boolean right;
    boolean up;
    boolean down;
    double leftStick;
    double rightStick;

    @Override
    public void runOpMode() throws InterruptedException {
//        int xVal, yVal, zVal = 0;     // Gyro rate Values
//        int heading = 0;              // Gyro integrated heading
//        int angleZ = 0;


        _motorA = hardwareMap.dcMotor.get("motor_a");
        _motorB = hardwareMap.dcMotor.get("motor_b");
        _motorC = hardwareMap.dcMotor.get("motor_c");
        _motorD = hardwareMap.dcMotor.get("motor_d");
        _MotorE = hardwareMap.dcMotor.get("motor_e");
        _MotorF = hardwareMap.dcMotor.get("motor_f");



//        _gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
//        _gyro.calibrate();

//        while (_gyro.isCalibrating())  {
//            Thread.sleep(50);
//            idle();
//        }
//
        waitForStart();
//
//        _gyro.resetZAxisIntegrator();

        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            leftStick = gamepad1.left_stick_y;
            rightStick = -gamepad1.right_stick_y;

            left = gamepad1.dpad_left;
            right = gamepad1.dpad_right;

            up = gamepad1.dpad_up;
            down = gamepad1.dpad_down;

//            xVal = _gyro.rawX();
//            yVal = _gyro.rawY();
//            zVal = _gyro.rawZ();
//
//            heading = _gyro.getHeading();
//            angleZ  = _gyro.getIntegratedZValue();
//
//            telemetry.addData("xVal", xVal);
//            telemetry.addData("yVal", yVal);
//            telemetry.addData("zVal", zVal);
//
//            telemetry.addData("heading", heading);
//            telemetry.addData("angleZ", angleZ);
//
//            telemetry.addData("left", left);
//            telemetry.addData("right", right);
//            telemetry.update();


            if (left) {
                _motorA.setPower(-1);
                _motorB.setPower(1);
                _motorC.setPower(-1);
                _motorD.setPower(1);

            } else if (right) {
                _motorA.setPower(1);
                _motorB.setPower(-1);
                _motorC.setPower(1);
                _motorD.setPower(-1);

            }

            else if (down) {
                _MotorE.setPower(1);
                _MotorF.setPower(-1);

            }
            else if (up) {
                _MotorE.setPower(-1);
                _MotorF.setPower(1);
            }

            else {
                _motorA.setPower(leftStick);
                _motorB.setPower(leftStick);
                _motorC.setPower(rightStick);
                _motorD.setPower(rightStick);
                _MotorE.setPower(0);
                _MotorF.setPower(0);
            }



            sleep(10);
            idle();


        }
    }
}

