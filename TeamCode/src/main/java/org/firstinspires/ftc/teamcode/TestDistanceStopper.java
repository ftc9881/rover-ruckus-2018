package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "TestDistanceStopper", group = "Test")
//@Disabled
public class TestDistanceStopper extends OctobotMain {
    /*
      * Main loop
      */
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the hardware
         */

        initialize();

        // wait for the start button to be pressed.
        waitForStart();

        double initialHeading = getCurrentHeading();

        drive(new IMUDriver(0, .45, _imu1, .04, initialHeading, RobotControl.convertInchesStrafe(36),
                        new StopperIF() {

                            @Override
                            public void start() {

                            }

                            @Override
                            public boolean keepGoing(int position) {
                                RobotLog.d("TestDistanceStopper::keepGoing()::distance %f %f", _irSensorLeft.getDistance(), _irSensorRight.getDistance());
                                RobotLog.d("TestDistanceStopper::keepGoing()::voltage %f %f", _irSensorLeft.getVoltage(), _irSensorRight.getVoltage());
                                return true;
                            }

                            @Override
                            public void finish() {

                            }
                        }
                ), true, true
        );
    }

}
