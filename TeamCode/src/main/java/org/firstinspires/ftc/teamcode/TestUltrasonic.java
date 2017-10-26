package org.firstinspires.ftc.teamcode;


/**
 * Created by Olavi Kamppari on 10/29/2015.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "TestUltrasonic", group = "test")
@Disabled
public class TestUltrasonic extends OpMode {
    private DataLogger              dl;
    private Wire                    ds;                     // Distance Sensor
    private int                     readCount = 0;
    private int                     distance;
    private long                    pingTime;

    public void init() {
        ds          = new Wire(hardwareMap,"us_front",0xE0);
    }

    public void start() {
        state   = States.SEND_PING;
    }

    private enum States{SEND_PING,ACK_PING,WAIT_REPLY,READ_RESPONSE};

    private States state;

    @Override
    public void loop() {
        telemetry.addData("sate", state);

        switch (state) {
            case SEND_PING:
                ds.write(0x51, null);                                    // Send ping
                state = States.ACK_PING;
                break;
            case ACK_PING:
                if (ds.responseCount() > 0) {
                    ds.getResponse();
                    if (ds.isWrite()) {
                        pingTime = System.currentTimeMillis();
                        state = States.WAIT_REPLY;
                    }
                }
                break;
            case WAIT_REPLY:
                if ((System.currentTimeMillis() - pingTime) > 100) {
                    ds.requestFrom(0, 2);                        // Request response
                    state = States.READ_RESPONSE;
                }
                break;
            case READ_RESPONSE:
                if (ds.responseCount() > 0) {
                    ds.getResponse();
                    if (ds.isRead()) {
                        long micros = ds.micros();

                        distance = ds.readHL();

                        readCount++;
                        telemetry.addData("Count", readCount);
                        telemetry.addData("Time", micros / 1000);
                        telemetry.addData("cm", distance);

                        RobotLog.d("TestUltrasonic::readCount: " + readCount);
                        RobotLog.d("TestUltrasonic::micros: " + micros);
                        RobotLog.d("TestUltrasonic::distance: " + distance);

                        state = States.SEND_PING;
                    }
                }
                break;

        }

        telemetry.update();
    }

    public void stop() {
        ds.close();
    }
}
