package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 1/26/2016.
 */
public class BasketBotAutonomousTest extends BasketBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();
        initializeSensors();

        waitForStart();

        moveForward(convertInches(72), .25f);

    }
}
