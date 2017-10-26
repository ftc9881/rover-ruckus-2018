package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 1/26/2016.
 */
public class BasketBotAutonomousRightClimbers extends BasketBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();
        initializeSensors();

        waitForStart();

        moveForward(convertInches(30), .25f);
        turn(45, .25f);
        moveForward(convertInches(74), .4f);

        waitOneFullHardwareCycle();
        int currentHeading = _gyroSensor.getHeading();
        turn(90 - currentHeading, .25f);

        dumpClimbers();
    }
}
