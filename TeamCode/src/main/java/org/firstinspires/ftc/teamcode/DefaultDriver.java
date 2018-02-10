package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/19/2017.
 */
public class DefaultDriver extends DefaultStopper implements DriverIF {
    public DefaultDriver(StopperIF stopper) {
        super(stopper);
    }

    @Override
    public Steerage getSteerage() {
        return null;
    }
}
