package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by ftc on 2/19/2017.
 */
public class VuforiaBeaconStopper extends DefaultDriver {
    private VuforiaTrackableDefaultListener _beacon;
    private double _min;
    private double _max;

    public VuforiaBeaconStopper(VuforiaTrackableDefaultListener beacon, double min, double max, DriverIF driver) {
        super(driver);

        _beacon = beacon;
        _min = min;
        _max = max;
    }

    @Override
    public Steerage getSteerage() {
        Steerage steerage = super.getSteerage();

        if(steerage == null) {
            return Steerage.createStationary();
        }
        else {
            return steerage;
        }
    }

    @Override
    public boolean keepGoing(int position) {
        boolean keepGoing = super.keepGoing(position);

        if(keepGoing) {
            OpenGLMatrix rawPose = _beacon.getRawPose();

            if(rawPose != null) {
                VectorF angles = VuforiaUtil.anglesFromTarget(rawPose);
                VectorF translation = _beacon.getPose().getTranslation();

                /*
                    Z is distance away
                    Y is left and right
                */

                double theta = angles.get(1);
                double offset = translation.get(1);
                double distance = translation.get(2);

                RobotLog.d("VuforiaBeaconStopper::keepGoing::theta: " + theta);
                RobotLog.d("VuforiaBeaconStopper::keepGoing::offset: " + offset);
                RobotLog.d("VuforiaBeaconStopper::keepGoing::distance: " + distance);
                RobotLog.d("VuforiaBeaconStopper::keepGoing::min: " + _min);
                RobotLog.d("VuforiaBeaconStopper::keepGoing::max: " + _max);

                if((!Double.isNaN(_min) && offset >= _min) || (!Double.isNaN(_max) && offset <= _max)) {
                    keepGoing = false;
                }
            }
            else {
                RobotLog.d("VuforiaBeaconStopper::keepGoing::no pose");
            }
        }

        return keepGoing;
    }
}
