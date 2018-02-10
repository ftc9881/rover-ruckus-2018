package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Created by dkrider on 10/27/2017.
 */

public class MotorRunner implements Runnable {
    private Thread _thread;

    protected DcMotor _motor;
    protected double _power = 0;
    protected int _minPosition = Integer.MIN_VALUE;
    protected int _maxPosition = Integer.MAX_VALUE;
    protected StopperIF _stopper = null;

    MotorRunner(DcMotor motor, double power, StopperIF stopper) {
        _motor = motor;
        _power = power;
        _stopper = stopper;
    }

    public void startMotor() {
        RobotLog.d("MotorRunner::startMotor()::A");

        if(_thread == null) {
            RobotLog.d("MotorRunner::startMotor()::B");
            _thread = new Thread(this);
        }

        RobotLog.d("MotorRunner::startMotor()::C");

        _thread.start();

        RobotLog.d("MotorRunner::startMotor()::D");
    }

    @Override
    public void run() {
        RobotLog.d("MotorRunner::run()::A");

        _stopper.start();

        _motor.setPower(_power);

        RobotLog.d("MotorRunner::run()::B");

        int position = _motor.getCurrentPosition();

        if(_stopper.keepGoing(position)) {
            do {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                }

                position = _motor.getCurrentPosition();
            } while (_stopper.keepGoing(position));
        }

        RobotLog.d("MotorRunner::run()::C");

        _motor.setPower(0);

        _stopper.finish();

        RobotLog.d("MotorRunner::run()::D");
    }
}
