package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class ConstantConfig {
    public static boolean debug = false;
    public static boolean blueSide = false;

    public static double driveKp = 2.3;
    public static double driveKi = 0.0;
    public static double driveKd = 0.003;

    public static double flywheelKp = 0.08; //0.08
    public static double flywheelKi = 0.05; //0.05
    public static double flywheelKd = 0.002; //0.002
    public static double flywheelKv = 0.01; //0.01
    public static double flywheelKs = 0.0055; //0.004

    public static final Pose2D goalPoseBlue = new Pose2D(DistanceUnit.INCH, -63, 65,
            AngleUnit.DEGREES, 0);
    public static final Pose2D goalPoseRed = new Pose2D(DistanceUnit.INCH, 63, 65,
            AngleUnit.DEGREES, 0);

    public static final Pose2D initialPoseBlue = new Pose2D(DistanceUnit.INCH, -12, -63,
            AngleUnit.DEGREES, 90);
    public static final Pose2D initialPoseRed = new Pose2D(DistanceUnit.INCH, 12, -63,
            AngleUnit.DEGREES, 90);

    public static double[] powerVals = {40, 42, 44, 85};

}