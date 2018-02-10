package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name = "OctobotCubeFinder", group = "octobot")
public class OctobotCubeFinder extends OctobotMain
{

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Motors and Sensors");
        telemetry.update();

        initialize();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        _servoGrabberBlue.setPosition(1);
        _servoGrabberRed.setPosition(1);
        sleep(750);

        grabCubeFromPile(getCurrentHeading(), false);


    }

}
