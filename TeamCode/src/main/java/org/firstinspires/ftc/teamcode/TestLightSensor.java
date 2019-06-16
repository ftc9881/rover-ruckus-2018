package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.RobotLog;

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

@TeleOp(name = "Test Light Sensor", group = "Test")
public class TestLightSensor extends LinearOpMode {
    /* Public OpMode members. */
    public DigitalChannel _led0 = null;
    public DigitalChannel _led1 = null;
    public AnalogInput _analog0 = null;
    public AnalogInput _analog1 = null;

    @Override
    public void runOpMode() throws InterruptedException {
//        _led0 = hardwareMap.digitalChannel.get("led_0");
//        _led0.setMode(DigitalChannel.Mode.OUTPUT);
//
//        _led1 = hardwareMap.digitalChannel.get("led_1");
//        _led1.setMode(DigitalChannel.Mode.OUTPUT);
//
//        _analog0 = hardwareMap.analogInput.get("analog_0");
        _analog1 = hardwareMap.analogInput.get("analog_1");

        waitForStart();
//
        while (opModeIsActive()) {
            boolean buttonA = gamepad1.a;
            boolean buttonB = gamepad1.b;

//            _led0.setState(buttonA);
//            _led1.setState(buttonB);
//
//            double voltage0 =  _analog0.getVoltage();
            double voltage1 =  _analog1.getVoltage();

            //telemetry.addData("analog0", voltage0);
            telemetry.addData("analog1", voltage1);
            telemetry.update();

            //RobotLog.d("analog0: " + voltage0);
            RobotLog.d("analog1: " + voltage1);
            sleep(10);
            idle();
        }
    }
}

