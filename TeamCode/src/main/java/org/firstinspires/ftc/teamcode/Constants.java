package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Constants {
    public static boolean DEBUG = false;
    public static boolean BLUE_SIDE = false;
    public static boolean IS_FIELD_CENTRIC = true;
    public static boolean USE_MAHALANOBIS_DISTANCE = false;

    public static double SLOW_SPEED_LT = 0.35;
    public static double LT_DEAD_ZONE = 0.4;

    public static double RIGHT_STICK_DEAD_ZONE = 0.03;

    public static double AUTO_FAR_POWER = 63;

    public static double DRIVE_KP = 2.3;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.003;

    public static double DRIVE_PID_BUFFER_MS = 150;

    public static double FLYWHEEL_KP = 0.08; //0.08
    public static double FLYWHEEL_KI = 0.05; //0.05
    public static double FLYWHEEL_KD = 0.002; //0.002
    public static double FLYWHEEL_KV = 0.01; //0.01
    public static double FLYWHEEL_KS = 0.0055; //0.004

    public static double MAX_FLYWHEEL_KV = 0.014;
    public static double MIN_FLYWHEEL_KV = 0.006;

    public static double VOLTS_NOMINAL = 12.5;

    public static double LAUNCHER_ENCODER_PER_REV = 28.0;

    public static double LIFT_SERVO_MIN = 0.1;
    public static double LIFT_SERVO_MAX = 0.24;
    public static double LIFT_SERVO_UP = 0.0;
    public static double LIFT_SERVO_DOWN = 1.0;
    public static double LIFT_SERVO_FLICK_TIME = 120.0;

    public static double[] POWER_STEPS = {56, 58, 63, 85}; //{40, 42, 44, 85} old one idk why no work

    public static final Pose2D GOAL_POSE_BLUE = new Pose2D(DistanceUnit.INCH, -63, 62,
            AngleUnit.DEGREES, 0);
    public static final Pose2D GOAL_POSE_RED = new Pose2D(DistanceUnit.INCH, 63, 62,
            AngleUnit.DEGREES, 0);

    public static final Pose2D INITIAL_POSE_BLUE = new Pose2D(DistanceUnit.INCH, -13, -35,
            AngleUnit.DEGREES, 90);
    public static final Pose2D INITIAL_POSE_RED = new Pose2D(DistanceUnit.INCH, 13, -35,
            AngleUnit.DEGREES, 90);

    public static final Vector2d PARK_POSE_BLUE = new Vector2d(33, -39);
    public static final Vector2d PARK_POSE_RED = new Vector2d(-33, -39);

    public static class HWMap {
        public static String FL_MOTOR = "front_left_motor";
        public static String FR_MOTOR = "front_right_motor";
        public static String BL_MOTOR = "back_left_motor";
        public static String BR_MOTOR = "back_right_motor";

        public static String LAUNCHER_MOTOR = "launcher_motor";
        public static String INTAKE_MOTOR = "intake_motor";
        public static String LIFT_SERVO = "lift_servo";

        public static String ODO = "odo";
        public static String IMU = "imu";

        public static String LIMELIGHT = "limelight";
        public static String COLOR_SENSOR = "color_sensor";
    }

    public static class llPipelines {
        public static int LOCALIZATION = 0;
        public static int APRIL_TAG_DETECTION = 1;
    }

}