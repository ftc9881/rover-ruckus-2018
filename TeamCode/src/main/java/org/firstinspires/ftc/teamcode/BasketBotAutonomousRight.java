package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 1/26/2016.
 */
public class BasketBotAutonomousRight extends BasketBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();
        initializeSensors();

        waitForStart();

        scooper(1);
        moveForward(convertInches(20), .25f);
        turn(40, .25f);
        moveForward(convertInches(48), .25f);
        turn(88, .25f);
        moveForward(convertInches(25), 75f);
        stopMotors();
    }
}
