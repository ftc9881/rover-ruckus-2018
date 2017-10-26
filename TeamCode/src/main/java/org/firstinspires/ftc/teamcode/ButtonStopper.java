package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/2/2017.
 */

class ButtonStopper extends DefaultDriver {
    double _power;

    DigitalChannel _pusherButton = null;

    public ButtonStopper(DigitalChannel pusherButton, DriverIF driver) {
        super(driver);

        _pusherButton = pusherButton;
    }

    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            keepGoing = !_pusherButton.getState();
        }

        return keepGoing;
    }
}
