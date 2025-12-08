package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

public class LimelightLocalization {
    /**
     * Plug Limelight into Computer and set values for offsets
     */
    private Limelight3A limelight;

    public void init(HardwareMap hwMap, int pipeline) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
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

    public Pose2D get2DLocation(Double heading) {
        limelight.updateRobotOrientation(heading);
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose3D = llResult.getBotpose_MT2();
            Position pose = botPose3D.getPosition();
            YawPitchRollAngles yawPitchRollAngles = botPose3D.getOrientation();
            Pose2D botPose2D = new Pose2D(DistanceUnit.INCH, pose.x, pose.y,
                    AngleUnit.RADIANS, yawPitchRollAngles.getYaw(AngleUnit.RADIANS));
            return botPose2D;
        } else {
            return null;
        }
    }

    public double getDistance(double ta) {
        double scale = 30665.96; //change ts
        double distance = scale / ta;
        return distance;
    }

    public byte scanObelisk() {
        LLResultTypes.FiducialResult tag = (LLResultTypes.FiducialResult) limelight.getLatestResult().getFiducialResults();

        return (byte) tag.getFiducialId();
    }

    public void updateTelemetry(Telemetry telemetry) {
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        multiTelemetry.addLine("===== Limelight Localization Telemetry =====");
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            multiTelemetry.addData("Limelight Status", limelight.getStatus());
            multiTelemetry.addData("Has Valid Result", result != null && result.isValid());
            
            if (result != null && result.isValid()) {
                Pose3D pose3D = result.getBotpose_MT2();
                if (pose3D != null) {
                    Position pos = pose3D.getPosition();
                    multiTelemetry.addData("X (in)", pos.x * 39.3701); // meters to inches
                    multiTelemetry.addData("Y (in)", pos.y * 39.3701);
                    multiTelemetry.addData("Z (in)", pos.z * 39.3701);
                }
            }
        } else {
            multiTelemetry.addData("Limelight Status", "NOT INITIALIZED");
        }

        multiTelemetry.update();
    }
}
