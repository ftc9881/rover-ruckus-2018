package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 2/6/2018.
 */

interface StopperIF {

    void start();

    /**
     * Returns true if we should keep going
     * @return
     */
    boolean keepGoing(int position);

    void finish();
}
