package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.security.KeyStore;

@TeleOp(name = "OctobotTeleOp", group = "octobot")
public class OctobotTeleOp extends OctobotMain {
    static final double GRIPPY_POWER = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
            Initialize all sensors, motors, servos, etc.
         */

        initialize();

        /*
            Initialize Vuforia
        */

//        initializeVuforia();

        /*
            Load "wilhelm" sound file
         */

       // MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.wilhelm);

        waitForStart();

        /*
            Start Vuforia navigaiton
         */

//        startVuforia();

        /*
            Initialize various state variables
         */

        boolean gripMode = false;
        boolean rightServoGrip = true;
        boolean reverseMode = false;
        boolean scooperrunning = false ;

       // boolean prevPusherButtonState = false;

        while (opModeIsActive()) {
            int positionA = _motorA.getCurrentPosition();
            int positionB = _motorB.getCurrentPosition();
            int positionC = _motorC.getCurrentPosition();
            int positionD = _motorD.getCurrentPosition();

            RobotLog.d("Motor Position " + positionA + " " + positionB + " " + positionC + " " + positionD);
            telemetry.addData("Motor Posistion", positionA + " " + positionB + " " + positionC + " " + positionD);

            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;

            /*
                Shoot the particle
             */

            if (rightBumper) {
                _motorSpinner.setPower(.2);
            } else if (leftBumper) {
                _motorSpinner.setPower(-.2);
            } else {
                _motorSpinner.setPower(0);
            }

            rightServoGrip = !rightServoGrip;

            if(rightBumper || leftBumper) {
                telemetry.addLine("Spinner Active");
            }

//            if(gripMode) {
//                _servoRight.setPower(1);
//                _servoRight.setDirection(DcMotorSimple.Direction.REVERSE);
//                _servoLeft.setPower(1);
//                _servoLeft.setDirection(DcMotorSimple.Direction.FORWARD);

//                if(rightServoGrip) {
//                    _servoRight.setPower(1);
//                    _servoRight.setDirection(DcMotorSimple.Direction.REVERSE);
//                    _servoLeft.setPower(1);
//                    _servoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//                }
//                else {
//                    _servoLeft.setPower(1);
//                    _servoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//                    _servoRight.setPower(1);
//                    _servoRight.setDirection(DcMotorSimple.Direction.FORWARD);
//                }
//
//                rightServoGrip = !rightServoGrip;
//            }

            telemetry.addData("heading", getCurrentHeading());

            telemetry.addData("Button 0 ", _button0.getState());

            telemetry.addData("Button 1 ", _button1.getState());

            telemetry.addData("Button 2 ", _button2.getState());

            telemetry.addData("Button 3 ", _button3.getState());

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean right = gamepad1.dpad_right;
            boolean left = gamepad1.dpad_left;

            if (right && !_button1.getState()) {
                _motorSlide.setPower(-1);
            } else if (left && !_button0.getState()) {
                _motorSlide.setPower(1);
            } else {
                _motorSlide.setPower(0);
            }

            if (right || left) {
                telemetry.addLine("Slide Active");
            } else {
                telemetry.addLine("Slide Inactive");
            }

            if (gamepad1.y) {
                _servoTop.setPosition(0);
            }
            else if (gamepad1.b) {
                _servoTop.setPosition(1);
            }

            if(gamepad1.x) {
                _servoBottom.setPosition(0);
            }
            else if (gamepad1.a) {
                _servoBottom.setPosition(1);
            }


            if (up || down) {
                telemetry.addLine("Lift Active");
            } else {
                telemetry.addLine("Lift Inactive");
            }

//           if(scooperrunning) {
//               _motorScooper.setPower(-1);
//           }
//            else {
//               _motorScooper.setPower(0);
//           }

            if (gamepad1.b) {
                gripMode = true;
            }

            telemetry.addLine("104");



            telemetry.addData("Reverse Mode ", reverseMode);

            float leftX = gamepad1.left_stick_x;
            float leftY = gamepad1.left_stick_y;

            float rightX = gamepad1.right_stick_x;
            float rightY = gamepad1.right_stick_y;

            float temp;

            reverseMode = true;

            if (reverseMode) {
                temp = -1 * leftX;
                leftX = -1 * rightX;
                rightX = temp;

                temp = -1 * leftY;
                leftY = -1 * rightY;
                rightY = temp;
            }

            if (up && !_button2.getState()) {
                _motorLift.setPower(1);
            }
            else if (down && !_button3.getState()) {
                _motorLift.setPower(-.2);
            } else{
                _motorLift.setPower(.05);
            }

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

            /*
                If the pusher button is pressed then make a sound
             */

            //boolean pusherButtonState = _pusherButton.getState();

            /*if (prevPusherButtonState != pusherButtonState) {
                if (pusherButtonState) {
                    mediaPlayer.start();
                }
            }
*/
//            OpenGLMatrix lastLocation = getVuforiaLocation();
//
//            /**
//             * Provide feedback as to where the robot was last located (if we know).
//             */
//            if (lastLocation != null) {
//                telemetry.addData("Vuforia", VuforiaUtil.formatOpenGLMatrix(lastLocation));
//                RobotLog.d("Vuforia::" + VuforiaUtil.formatOpenGLMatrix(lastLocation));
//            } else {
//                telemetry.addData("Vuforia", "Unknown");
//            }

            telemetry.update();

            sleep(10);
            idle();
        }

        /*
            Stop Vuforia navigaiton
         */

        stopVuforia();
    }
}

