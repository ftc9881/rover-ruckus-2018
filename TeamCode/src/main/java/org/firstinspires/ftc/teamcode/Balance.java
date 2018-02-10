package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name="Balance", group= "octobot")
@Disabled

public class Balance extends OctobotMain {

    Orientation angles;
    float pitchAngle;
    float rollAngle;


    @Override
    public void runOpMode() throws InterruptedException {




        initialize();
        waitForStart();

        Orientation angles;
        Acceleration gravity;



        while(opModeIsActive()) {
            float leftX = gamepad1.left_stick_x;
            float leftY;

            float rightX = gamepad1.right_stick_x;
            float rightY;

            float multiplier;

            angles   = _imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            pitchAngle = 0;


//            telemetry.addData("IMU angle", _imu.getAngularOrientation());
            telemetry.addData("IMU1 angle", _imu1.getAngularOrientation());
//            telemetry.addData("IMU2 angle", _imu2.getAngularOrientation());
//            telemetry.addData("IMU orientation", _imu.getQuaternionOrientation());
            telemetry.addData("IMU1 orientation", _imu1.getQuaternionOrientation());
//            telemetry.addData("IMU2 orientation", _imu2.getQuaternionOrientation());

//            telemetry.addData("cal",_imu.getCalibrationStatus());

            telemetry.addData("pitch", angles.thirdAngle);

            if ((angles.thirdAngle) > 0){
                pitchAngle = 180-angles.thirdAngle;
            }
            else if(angles.thirdAngle < 0){
                pitchAngle = angles.thirdAngle + 180;
                pitchAngle = -pitchAngle;
            }
            telemetry.addData("pitch",pitchAngle);

            rollAngle = -angles.secondAngle;

            multiplier = .075f;

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

            double motorAPower = RobotControl.convertStickToPower(frontRight);
            double motorBPower = RobotControl.convertStickToPower(rearRight);
            double motorCPower = RobotControl.convertStickToPower(frontLeft);
            double motorDPower = RobotControl.convertStickToPower(rearLeft);

            RobotLog.d("Motor Power " + motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);
            telemetry.addData("Motor Power", motorAPower + " " + motorBPower + " " + motorCPower + " " + motorDPower);

            _motorA.setPower(motorAPower);
            _motorB.setPower(motorBPower);
            _motorC.setPower(motorCPower);
            _motorD.setPower(motorDPower);

            telemetry.update();

            idle();
        }


    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
