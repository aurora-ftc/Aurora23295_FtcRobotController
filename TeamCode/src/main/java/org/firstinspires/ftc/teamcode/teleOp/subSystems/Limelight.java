package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teleOp.Constants;

import java.util.ArrayList;
import java.util.List;

public class Limelight {
    /**
     * Plug Limelight into Computer and set values for offsets
     */
    private final Limelight3A limelight;
    private byte obeliskID;
    private Mosaic mosaic;
    private List<LLResultTypes.FiducialResult> obeliskList = new ArrayList<>();

    public Limelight(HardwareMap hwMap, int pipeline) {
        limelight = hwMap.get(Limelight3A.class, Constants.HWMap.LIMELIGHT);
        limelight.pipelineSwitch(pipeline); //0 is the normal
        limelight.start();
    }

    public Pose3D getLocation(Double heading) {
        limelight.updateRobotOrientation(heading);
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            return botPose;
        } else {
            return null;
        }
    }

    public Pose2D get2DLocationMT1() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose3D = llResult.getBotpose();
            Position pose = botPose3D.getPosition();
            YawPitchRollAngles yawPitchRollAngles = botPose3D.getOrientation();
            Pose2D botPose2D = new Pose2D(DistanceUnit.METER, pose.x, pose.y,
                    AngleUnit.RADIANS, yawPitchRollAngles.getYaw(AngleUnit.RADIANS));
            return botPose2D;
        } else {
            return null;
        }
    }

    public void changePipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public Pose2D get2DLocation(Double heading) {
        limelight.updateRobotOrientation(heading);
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose3D = llResult.getBotpose_MT2();
            Position pose = botPose3D.getPosition();
            YawPitchRollAngles yawPitchRollAngles = botPose3D.getOrientation();
            Pose2D botPose2D = new Pose2D(DistanceUnit.METER, pose.x, pose.y,
                    AngleUnit.RADIANS, yawPitchRollAngles.getYaw(AngleUnit.RADIANS));
            return botPose2D;
        } else {
            return null;
        }
    }

    public Mosaic scanObelisk() {
        LLResult result = limelight.getLatestResult();

        // If no frame or invalid frame, return UNKNOWN
        if (result == null || !result.isValid()) {
            mosaic = Mosaic.UNKNOWN;
            return mosaic;
        }

        obeliskList = result.getFiducialResults();

        // No tags seen → UNKNOWN
        if (obeliskList == null || obeliskList.isEmpty()) {
            mosaic = Mosaic.UNKNOWN;
            return mosaic;
        }

        // Get the first detected tag
        obeliskID = (byte) obeliskList.get(0).getFiducialId();

        // Convert ID → PPG/GPP/PGP/UNKNOWN
        mosaic = Mosaic.fromId(obeliskID);

        return mosaic;
    }


    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Limelight Status", limelight.getStatus());
        telemetry.addLine();

        telemetry.addData("Obelisk", obeliskID);
    }

    public double getDistance(double ta) {
        double scale = 30665.96; //change ts
        double distance = scale / ta;
        return distance;
    }
}