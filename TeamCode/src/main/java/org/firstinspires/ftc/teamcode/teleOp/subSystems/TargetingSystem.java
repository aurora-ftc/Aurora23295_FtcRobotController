package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TargetingSystem {
    Limelight3A limelight;
    byte obelisk;

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

    }

    public double target(byte targetID) {
        LLResultTypes.FiducialResult tag = (LLResultTypes.FiducialResult) limelight.getLatestResult().getFiducialResults();

        if (targetID == (byte) tag.getFiducialId()) {
            //TODO: Get difference between robot heading and the target tag. Essentially minimize "X".
            return 1.0;
        } else {
            return 0.0;
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        multiTelemetry.addLine("===== Targeting System Telemetry =====");
        multiTelemetry.addData("Limelight Status", limelight.getStatus());
        multiTelemetry.addData("Obelisk ID", obelisk);

        multiTelemetry.update();
    }
}