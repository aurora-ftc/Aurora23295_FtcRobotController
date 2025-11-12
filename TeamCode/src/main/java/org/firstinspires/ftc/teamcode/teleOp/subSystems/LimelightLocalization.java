package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

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
        double distance = scale/ta;
        return distance;
    }
}
