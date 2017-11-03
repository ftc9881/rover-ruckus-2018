package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Balance", group= "octobot")


public class Balance extends OctobotMain {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("IMU angle", _imu.getAngularOrientation());
            telemetry.addData("IMU1 angle", _imu1.getAngularOrientation());
            telemetry.addData("IMU2 angle", _imu2.getAngularOrientation());
            telemetry.addData("IMU orientation", _imu.getQuaternionOrientation());
            telemetry.addData("IMU1 orientation", _imu1.getQuaternionOrientation());
            telemetry.addData("IMU2 orientation", _imu2.getQuaternionOrientation());

            telemetry.addData("cal",_imu.getCalibrationStatus());


            telemetry.update();

            sleep(10);
            idle();
        }


    }
}
