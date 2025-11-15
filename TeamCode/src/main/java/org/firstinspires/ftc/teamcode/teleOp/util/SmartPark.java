package org.firstinspires.ftc.teamcode.teleOp.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;

public class SmartPark {

    private final MecanumDrive drive;
    private final org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive dwive;

    public SmartPark(MecanumDrive drive, org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive dwive) {
        this.drive = drive;
        this.dwive = dwive;
    }

    public TrajectoryActionBuilder buildParkAction(Vector2d target, Telemetry tele) {
        drive.updateOdo();
        Pose2D odoPos = drive.getOdoPosition();

        double robotX = odoPos.getX(DistanceUnit.INCH);
        double robotY = odoPos.getY(DistanceUnit.INCH);
        double currentHeading = odoPos.getHeading(AngleUnit.DEGREES);

        double angleToTarget = Math.toDegrees(Math.atan2(target.y - robotY, target.x - robotX));
        double[] cardinalAngles = {0.0, 90.0, 180.0, 270.0};
        double bestAngle = cardinalAngles[0];
        double minDiff = Math.abs(angleDiff(angleToTarget, bestAngle));

        for (double a : cardinalAngles) {
            double diff = Math.abs(angleDiff(angleToTarget, a));
            if (diff < minDiff) {
                minDiff = diff;
                bestAngle = a;
            }
        }

        double diffToCurrent = angleDiff(bestAngle, currentHeading);
        boolean shouldReverse = Math.abs(diffToCurrent) < 90;

        Pose2d startPose = new Pose2d(robotX, robotY, Math.toRadians(currentHeading));

        TrajectoryActionBuilder builder;

        if (shouldReverse) {
            builder = dwive.actionBuilder(startPose)
                    .setReversed(true)
                    .splineTo(target, Math.toRadians(bestAngle));
        } else {
            builder = dwive.actionBuilder(startPose)
                    .setReversed(false)
                    .splineTo(target, Math.toRadians(bestAngle));
        }

        tele.addLine("SmartPark trajectory built");
        tele.update();

        return builder; // return Trajectory (not executed or built yet)
    }

    private double angleDiff(double target, double current) {
        double diff = target - current;
        diff = ((diff + 180) % 360) - 180;
        return diff;
    }
}
