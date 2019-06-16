package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ftc on 2/2/2017.
 */

public interface DriverIF extends StopperIF {
    class Steerage {
        double _left;
        double _right;
        double _strafe;

        static final Steerage _stationary = new Steerage(0, 0, 0);

        Steerage(double left, double right, double strafe) {
            _left = left;
            _right = right;
            _strafe = strafe;
        }

        public Steerage(double power) {
            _left = power;
            _right = power;
            _strafe = 0;
        }

        public double getLeft() {
            return _left;
        }

        public double getRight() {
            return _right;
        }

        public double getStrafe() {
            return _strafe;
        }

        static Steerage createPowerSteerage(double power, double steeringFactor, double strafePower) {
            double effectivePower;

            if(power > 0) {
                effectivePower = Math.max(power, Math.abs(strafePower));
            }
            else {
                effectivePower = Math.max(-power, Math.abs(strafePower));
            }

            double left = power - effectivePower * steeringFactor;
            double right =  power + effectivePower * steeringFactor;

            RobotLog.d("Steerage::createPowerSteerage::effectivePower: " + effectivePower);
            RobotLog.d("Steerage::createPowerSteerage::steeringFactor: " + steeringFactor);
            RobotLog.d("Steerage::createPowerSteerage::left: " + left);
            RobotLog.d("Steerage::createPowerSteerage::right: " + right);

            return new Steerage(left, right, strafePower);
        };

        public boolean equals(Object object) {
            return (object instanceof Steerage)
                    && (object != null)
                    && (((Steerage)object)._left == _left)
                    && (((Steerage)object)._right == _right)
                    && (((Steerage)object)._strafe == _strafe);
        }

        public String toString() {
            return "_left: " + _left + " _right: " + _right + " _strafe: " + _strafe;
        }

        public static Steerage createStationary() {
            return _stationary;
        }
    }

    /**
     * Return the power that should be used for the motors
     * @return
     */
    Steerage getSteerage();
}
