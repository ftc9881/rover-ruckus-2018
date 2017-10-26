package org.firstinspires.ftc.teamcode;

/**
 * Created by ftc on 2/2/2017.
 */

public interface DriverIF {
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
            double effectivePower = Math.max(power, strafePower);

            return new Steerage(power - effectivePower * steeringFactor, power + effectivePower * steeringFactor, strafePower);
        };

        public boolean equals(Object object) {
            return (object instanceof Steerage)
                    && (object != null)
                    && (((Steerage)object)._left == _left)
                    && (((Steerage)object)._right == _right)
                    && (((Steerage)object)._strafe == _strafe);
        }

        public static Steerage createStationary() {
            return _stationary;
        }
    }

    void start();

    /**
     * Return the power that should be used for the motors
     * @return
     */
    Steerage getSteerage();

    /**
     * Returns true if we should stop
     * @return
     */
    boolean keepGoing(int position);

    void finish();
}
