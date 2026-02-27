package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.content.Context;
import android.content.SharedPreferences;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.LimelightControl;
import org.firstinspires.ftc.teamcode.teleOp.util.Mosaic;

@Autonomous(name = "Scan Tag Auto")
public class ScanTagAuto extends LinearOpMode {

    private LimelightControl limelightControl;

    @Override
    public void runOpMode() {

        // Initialize LimelightControl
        limelightControl = new LimelightControl(hardwareMap, 1); // pipeline 0

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        Mosaic pattern = Mosaic.UNKNOWN;

        // Scan until we get a valid pattern
        while (opModeIsActive() && pattern == Mosaic.UNKNOWN) {
            pattern = limelightControl.scanObelisk();
            telemetry.addData("Pattern Detected", pattern.name());
            telemetry.update();

            sleep(50); // small delay to avoid spamming
        }

        // Save the detected pattern to SharedPreferences for TeleOp
        SharedPreferences prefs = hardwareMap.appContext
                .getSharedPreferences("RobotPrefs", Context.MODE_PRIVATE);
        prefs.edit().putString("detectedMosaic", pattern.name()).apply();

        telemetry.addData("Final Pattern", pattern.name());
        telemetry.update();

        sleep(3000); // pause to allow driver to see telemetry
    }
}
