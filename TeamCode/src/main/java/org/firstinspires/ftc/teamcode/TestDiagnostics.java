package org.firstinspires.ftc.teamcode;

import android.content.pm.ApplicationInfo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.RobotLog;

import java.text.SimpleDateFormat;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep() 
 * and a for loop.
 */
@TeleOp(name = "TestDiagnostics", group = "Test")
public class TestDiagnostics extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        try{
            ApplicationInfo ai = hardwareMap.appContext.getPackageManager().getApplicationInfo(hardwareMap.appContext.getPackageName(), 0);
            ZipFile zf = new ZipFile(ai.sourceDir);
            ZipEntry ze = zf.getEntry("classes.dex");
            long time = ze.getTime();
            String s = SimpleDateFormat.getInstance().format(new java.util.Date(time));
            zf.close();

            telemetry.addData("Build Time", s);
        }catch(Exception e){
        }

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            idle();
        }
    }

}
