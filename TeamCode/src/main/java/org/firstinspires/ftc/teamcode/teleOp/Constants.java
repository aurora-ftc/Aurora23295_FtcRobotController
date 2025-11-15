package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Constants {
    public static boolean debug = false;
    public static boolean blueSide = false;
    public static boolean fieldCentric = true;
    public static boolean useMahalanobisDistance = true;

    public static double liftServoMin = 0.1;
    public static double liftServoMax = 0.24;
    public static double liftServoUp = 0;
    public static double liftServoDown = 1;
    public static double liftServoFlickTimeMS = 120;

    public static double driveKp = 2.3;
    public static double driveKi = 0.0;
    public static double driveKd = 0.003;
    public static double drivePIDBufferMS = 160;

    public static double flywheelKp = 0.08; //0.08
    public static double flywheelKi = 0.05; //0.05
    public static double flywheelKd = 0.002; //0.002
    public static double flywheelKv = 0.01; //0.01
    public static double flywheelKs = 0.0055; //0.0055

    public static double maxFlywheelKv = 0.013;
    public static double minFlywheelKv = 0.007;

    public static double[] launcherPowerSteps = {56, 62, 85}; //{40, 42, 44, 85} {56, 58, 62, 65, 85}
    public static double launcherMotorEncoderTicks = 28.0;

    public static double voltsNormal = 12.5;

    public static double maxDriveSpeed = 1.0;
    public static double slowSpeedLT = 0.35;

    public static double rotateStickDeadZone = 0.03;

    public static double odoXOffsetMM = -41;
    public static double odoYOffsetMM = 0;

    public static GoBildaPinpointDriverRR.EncoderDirection odoXDirection =
            GoBildaPinpointDriverRR.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriverRR.EncoderDirection odoYDirection =
            GoBildaPinpointDriverRR.EncoderDirection.REVERSED;

    public static final Pose2D goalPoseBlue = new Pose2D(DistanceUnit.INCH, -63, 65,
            AngleUnit.DEGREES, 0);
    public static final Pose2D goalPoseRed = new Pose2D(DistanceUnit.INCH, 63, 65,
            AngleUnit.DEGREES, 0);

    public static final Pose2D initialPoseBlue = new Pose2D(DistanceUnit.INCH, -12, -63,
            AngleUnit.DEGREES, 90);
    public static final Pose2D initialPoseRed = new Pose2D(DistanceUnit.INCH, 13, -35,
            AngleUnit.DEGREES, 90);

    public static double intakeBlipStart = 150;
    public static double intakeBlipEnd = 500;

    //Only change last value
    public static double threeShotCycleBuffer = liftServoFlickTimeMS + intakeBlipEnd + 150;

    public static float revColorSensorGain = 10;

    public static class HardwareConfig {
        public static String frontLeftDriveMotor = "front_left_motor";
        public static String frontRightDriveMotor = "front_right_motor";
        public static String backLeftDriveMotor = "back_left_motor";
        public static String backRightDriveMotor = "back_right_motor";

        public static String launcherMotor = "launcher_motor";
        public static String intakeMotor = "intake_motor";
        public static String liftServo = "lift_servo";

        public static String odo = "odo";
        public static String imu = "imu";

        public static String limelight = "limelight";
        public static String colorSensor = "color_sensor";
    }
}