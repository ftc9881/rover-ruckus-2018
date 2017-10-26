package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

/**
 * Created by ftc on 2/18/2017.
 */

@TeleOp(name = "TestVuforia", group = "octobot")
@Disabled
public class TestVuforia extends OctobotMain {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing");
        telemetry.update();

        initialize();

        telemetry.addLine("Initializing Vuforia");
        telemetry.update();

        initializeVuforia();

        telemetry.addLine("Calibrating");
        telemetry.update();

//        calibrateSensors();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Starting Vuforia");
        telemetry.update();

        startVuforia();

        while (opModeIsActive()) {
            OpenGLMatrix lastLocation = getVuforiaLocation();

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
//                telemetry.addData("Pos", VuforiaUtil.formatOpenGLMatrix(lastLocation));
            } else {
//                telemetry.addData("Pos", "Unknown");
            }

            for (VuforiaTrackable trackable : _beacons) {
                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();


                if(listener.getPose() != null) {
                    VectorF angles = VuforiaUtil.anglesFromTarget(listener.getRawPose());
                    VectorF translation = listener.getPose().getTranslation();
                    /*
                        Z is distance away
                        Y is left and right
                     */
                    telemetry.addData(trackable.getName(), "angles: " + angles);
                    telemetry.addData(trackable.getName(), "translation: " + translation);
                }
                else {
                    telemetry.addData(trackable.getName(), "nope");
                }
            }

            telemetry.update();
            idle();
        }

    }


}
