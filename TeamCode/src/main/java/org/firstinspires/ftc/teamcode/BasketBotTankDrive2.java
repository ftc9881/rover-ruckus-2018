/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class BasketBotTankDrive2 extends OpMode {
	DcMotor _motorA;
	DcMotor _motorB;
	DcMotor _motorC;
	DcMotor _motorD;
	DcMotor _motorE;
	DcMotor _motorF;
	DcMotor _motorG;
	DcMotor _motorH;

	GyroSensor _gyroSensor;

	/**
	 * Constructor
	 */
	public BasketBotTankDrive2() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		_motorA = hardwareMap.dcMotor.get("motor_a");
		_motorA.setDirection(DcMotor.Direction.FORWARD);
		_motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

		_motorB = hardwareMap.dcMotor.get("motor_b");
		_motorB.setDirection(DcMotor.Direction.REVERSE);
		_motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

		_motorC = hardwareMap.dcMotor.get("motor_c");
		_motorC.setDirection(DcMotor.Direction.FORWARD);
		_motorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

		_motorD = hardwareMap.dcMotor.get("motor_d");
		_motorD.setDirection(DcMotor.Direction.FORWARD);
		_motorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

		_motorE = hardwareMap.dcMotor.get("motor_e");
		_motorE.setDirection(DcMotor.Direction.REVERSE);
		_motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

		_motorF = hardwareMap.dcMotor.get("motor_f");
		_motorF.setDirection(DcMotor.Direction.REVERSE);
		_motorF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

		_motorG = hardwareMap.dcMotor.get("motor_g");
		_motorG.setDirection(DcMotor.Direction.FORWARD);
		_motorG.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

		_motorH = hardwareMap.dcMotor.get("motor_h");
		_motorH.setDirection(DcMotor.Direction.FORWARD);
		_motorH.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

		_gyroSensor = hardwareMap.gyroSensor.get("gyro");
		_gyroSensor.calibrate();
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventlo op.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		double heading = _gyroSensor.getHeading();
		telemetry.addData("heading", "heading: " + String.format("%.2f", heading));

		// tank drive
		// note that if y equal -1 then joystick is pushed all of the way forward.
		float left = -gamepad1.right_stick_y;
		float right = -gamepad1.left_stick_y;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		float rightPower = (float) scaleInput(right);
		float leftPower = (float) scaleInput(left);

		// write the values to the motors
		_motorE.setPower(rightPower);
		_motorF.setPower(rightPower);
		_motorG.setPower(leftPower);
		_motorH.setPower(leftPower);

		telemetry.addData("right power", String.format("%.2f", left));
		telemetry.addData("left power", String.format("%.2f", left));

		float motorAPosition = _motorA.getCurrentPosition();
		float motorBPosition = _motorB.getCurrentPosition();
		float motorCPosition = _motorC.getCurrentPosition();
		float motorDPosition = _motorD.getCurrentPosition();
		float motorEPosition = _motorE.getCurrentPosition();
		float motorFPosition = _motorF.getCurrentPosition();
		float motorGPosition = _motorG.getCurrentPosition();
		float motorHPosition = _motorH.getCurrentPosition();

		telemetry.addData("motor A", "position: " + String.format("%.2f", motorAPosition));
		telemetry.addData("motor B", "position: " + String.format("%.2f", motorBPosition));
		telemetry.addData("motor C", "position: " + String.format("%.2f", motorCPosition));
		telemetry.addData("motor D", "position: " + String.format("%.2f", motorDPosition));
		telemetry.addData("motor E", "position: " + String.format("%.2f", motorEPosition));
		telemetry.addData("motor F", "position: " + String.format("%.2f", motorFPosition));
		telemetry.addData("motor G", "position: " + String.format("%.2f", motorGPosition));
		telemetry.addData("motor H", "position: " + String.format("%.2f", motorHPosition));

		boolean buttonA = gamepad1.a;
		boolean buttonB = gamepad1.b;
		boolean buttonX = gamepad1.x;
		boolean buttonY = gamepad1.y;

		boolean buttonUp = gamepad1.dpad_up;
		boolean buttonDown = gamepad1.dpad_down;
		boolean buttonLeft = gamepad1.dpad_left;
		boolean buttonRight = gamepad1.dpad_right;

		if (buttonUp) {
			_motorD.setPower(1);
		} else if (buttonDown) {
			_motorD.setPower(-1);
		} else {
			_motorD.setPower(0);
		}

		if (buttonY) {
			_motorC.setPower(.5);
		} else if (buttonX) {
			_motorC.setPower(-1);
		} else if (buttonA) {
			_motorC.setPower(0);
		}

		if (buttonLeft) {
			_motorA.setPower(1);
			_motorB.setPower(1);
		}
		else if (buttonRight){
			_motorA.setPower(-1);
			_motorB.setPower(-1);
		}
		else {
			_motorA.setPower(0);
			_motorB.setPower(0);
		}

		//if(buttonX) {
		//	_motorC.setPower(-1);
		//}
		//else {
		//	_motorC.setPower(0);
		//}
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}
	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.01, 0.03, 0.06, 0.07, 0.10, 0.13, 0.15,
				0.19, 0.23, 0.30, 0.43, 0.56, 0.68, 0.80, 0.91, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		
		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
