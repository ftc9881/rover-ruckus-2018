package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 2/2/2017.
 */

public interface TurnerIF extends StopperIF {
    /**
     * Return the power that should be used for the motors
     * @return
     */
    double getPower();

    /**
     * Return a number between -1 and 1 for the amount it should turn
     * @return
     */
    double getScaleFactor();
}
