package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 3/1/2018.
 */
@TeleOp(name = "ScanCubes", group = "test")
public class ScanCubes extends LinearOpMode {
    SharpDistanceSensor _irDistanceSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        _irDistanceSensor = new SharpDistanceSensor(hardwareMap.analogInput.get("ir_sensor"),
                0.035070719,
                0.013259056,
                -0.860930745
        );

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("irSensors",_irDistanceSensor.getDistance() + " " + _irDistanceSensor.getVoltage());
            telemetry.update();

            RobotLog.d("ScanCubes::runOpMode()::distance: " + _irDistanceSensor.getDistance());
            RobotLog.d("ScanCubes::runOpMode()::voltage: " + _irDistanceSensor.getVoltage());

            sleep(10);
        }
    }
}
