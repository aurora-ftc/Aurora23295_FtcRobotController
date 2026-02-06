package org.firstinspires.ftc.teamcode.jCode.SubSystems;

import static org.firstinspires.ftc.teamcode.jCode.Enumerators.Patterns.GPP;
import static org.firstinspires.ftc.teamcode.jCode.Enumerators.Patterns.PGP;
import static org.firstinspires.ftc.teamcode.jCode.Enumerators.Patterns.PPG;
import static org.firstinspires.ftc.teamcode.jCode.Enumerators.Patterns.UNKNOWN;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.jCode.Enumerators.Patterns;

public class Limelight {
    private Limelight3A limelight;
    private int targetID;
    private double dx;
    private LLResultTypes.FiducialResult tag;

    public void init(HardwareMap map) {
        limelight = map.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public Patterns getSeenTag() {
        limelight.stop();
        limelight.pipelineSwitch(0);
        limelight.start();
        int id = limelight.getLatestResult().getFiducialResults().get(0).getFiducialId();
        switch (id) {
            case 21:
                return PPG;
            case 22:
                return PGP;
            case 23:
                return GPP;
            default:
                return UNKNOWN;
        }
    }

    public void periodic() {
//        tag = limelight.getLatestResult().getFiducialResults().get(limelight.getLatestResult().get(0));
        target();
    }

    public double target() {
        dx = tag.getTargetXDegrees();
        return dx;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Limelight Control");
        telemetry.addData("Limelight status", "OK");
        telemetry.addData("Seen Tags", limelight.getLatestResult().getFiducialResults());
        telemetry.addData("Targeting Error", dx);
    }
}
