/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="TestWire", group="Test")  // @Autonomous(...) is the other common choice
@Disabled
public class TestWire extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    Wire _sonar1;
    Wire _sonar2;
    Wire _sonar3;
    long _pingTime;

    @Override
    public void runOpMode() {
        _sonar1 = new Wire(hardwareMap, "sonar1", 0xDC);
        _sonar2 = new Wire(hardwareMap, "sonar2", 0xDE);
        _sonar3 = new Wire(hardwareMap, "sonar3", 0xE0);


        telemetry.addData("Status", "1");
        telemetry.update();

        waitForStart();

        _sonar1.beginWrite(0x51);
        _sonar1.write(0);
        _sonar1.endWrite();

        _sonar2.beginWrite(0x51);
        _sonar2.write(0);
        _sonar2.endWrite();

        _sonar3.beginWrite(0x51);
        _sonar3.write(0);
        _sonar3.endWrite();

        _pingTime = System.currentTimeMillis();

        Wire sonars[] = new Wire[] { _sonar1, _sonar2, _sonar3 };
        Wire sonar = null;

        int distances[] = new int[3];
        int sonarIndex = -1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(System.currentTimeMillis() - _pingTime > 100) {
                ++sonarIndex;
                if(sonarIndex >= sonars.length) {
                    sonarIndex = 0;
                }

                sonar = sonars[sonarIndex];

                sonar.requestFrom(0, 2);
                sonar.beginWrite(0x51);
                sonar.write(0);
                sonar.endWrite();

                _pingTime = System.currentTimeMillis();
            }

            if(sonar != null && sonar.responseCount() > 0) {
                sonar.getResponse();
                if(sonar.isRead()) {
                    distances[sonarIndex] = sonar.readHL();
                }
            }

            telemetry.addData("Status", "Distances: " + distances[0] + " " + distances[1] + " " + distances[2]);

            telemetry.update();
        }
    }
}
