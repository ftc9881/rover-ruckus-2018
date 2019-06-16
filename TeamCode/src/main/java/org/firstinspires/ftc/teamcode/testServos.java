package org.firstinspires.ftc.teamcode;



import android.graphics.Color;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.qualcomm.robotcore.util.RobotLog;



import java.util.ArrayList;

import java.util.List;



@Autonomous(name = "testServos", group = "MaximumOverdrive")
@Disabled
public class testServos extends UnitBotMain {


    @Override

    public void runOpMode() throws InterruptedException {

        /*

            Initialize all sensors, motors, servos, etc.

         */


        initialize();

        waitForStart();

        int tvalue;

        tvalue = _motorStronkBoi.getCurrentPosition();

        _motorStronkBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(true) {
            _motorStronkBoi.setTargetPosition(tvalue);
            sleep(100);
        }
    }
}