package org.firstinspires.ftc.teamcode.teleOp.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import  org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

public class SmartPark {

    private MecanumDrive drive;
    private PinpointDrive driveRR;
    private Vector2d parkPose;

    public SmartPark(MecanumDrive drive, PinpointDrive driveRR, Vector2d parkPose) {
        this.drive = drive;
        this.driveRR = driveRR;
        this.parkPose = parkPose;
    }

    public Action buildParkAction() {
        drive.updateOdo();

        Pose2D odoPos = drive.getOdoPosition();

        double robotX = odoPos.getX(DistanceUnit.INCH);
        double robotY = odoPos.getY(DistanceUnit.INCH);
        double currentHeading = odoPos.getHeading(AngleUnit.DEGREES);

        double bestAngle = 90;

        double[] cardinalAngles = {0, 90, -180, -90};

        double minDiff = Double.MAX_VALUE;

        for (double angle : cardinalAngles) {
            double diff = Math.abs(angleDiff(angle, currentHeading));
            if (diff < minDiff) {
                minDiff = diff;
                bestAngle = angle;
            }
        }


        Pose2d startPose = new Pose2d(robotX, robotY, Math.toRadians(currentHeading));

        TrajectoryActionBuilder builder = driveRR.actionBuilder(startPose)
                .strafeToLinearHeading(parkPose, Math.toRadians(bestAngle));

        Action action = builder.build();

        return action;
    }

    private static double angleDiff(double target, double current) {
        double diff = target - current;
        diff = ((diff + 180) % 360) - 180;
        return diff;
    }
}
