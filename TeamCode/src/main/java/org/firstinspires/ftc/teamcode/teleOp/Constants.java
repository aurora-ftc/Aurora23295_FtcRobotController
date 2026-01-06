package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Constants {
    // Limelight Pipelines
    public static final int APRIL_TAG_DETECTION = 0;
    // Points for Auto-Lock
    public static final Pose2D GOAL_POSE_BLUE = new Pose2D(DistanceUnit.INCH, -63, 62, AngleUnit.DEGREES, 0);
    public static final Pose2D GOAL_POSE_RED = new Pose2D(DistanceUnit.INCH, 63, 62, AngleUnit.DEGREES, 0);
    // Default Start Positions
    public static final Pose2D INITIAL_POSE_BLUE = new Pose2D(DistanceUnit.INCH, -13, -35, AngleUnit.DEGREES, 90);
    public static final Pose2D INITIAL_POSE_RED = new Pose2D(DistanceUnit.INCH, 13, -35, AngleUnit.DEGREES, 90);
    public static final double MOSAIC_FLASH_INTERVAL = 0.5;
    public static int PUSH_SERVO_FLICK_TIME = 250; //IN MS
    // Easy Mode Switches
    public static boolean DEBUG = false;
    public static boolean BLUE_SIDE = false;
    public static boolean IS_FIELD_CENTRIC = true;
    // Drive PID
    public static double DRIVE_KP = 2.3;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.003;
    // Flywheel PID
    public static double FLYWHEEL_KP = 0.08;   // 0.08
    public static double FLYWHEEL_KI = 0.05;   // 0.05
    public static double FLYWHEEL_KD = 0.002;  // 0.002
    public static double FLYWHEEL_KV = 0.01;   // 0.01
    public static double FLYWHEEL_KS = 0.0055; // 0.004
    // Range for Battery Adjusted Kv
    public static double MAX_FLYWHEEL_KV = 0.014;
    public static double MIN_FLYWHEEL_KV = 0.006;
    // Ideal Volts
    public static double VOLTS_NOMINAL = 12.5;
    // Launcher Ticks Per Rev
    public static double LAUNCHER_ENCODER_PER_REV = 28.0;
    // Slow Speed Modifier
    public static double SLOW_SPEED_LT = 0.35;
    public static double[] POWER_STEPS = {25, 40, 42, 44, 85};
    public static int[] POSITIONS = {0, 1333, 2666, 667, 2000, 3333};
    public static float revColorSensorGain = 0.4f;
    /**
     * Rotary Selector System Configuration
     */
    public static double ROTARY_KP = 0.000055;
    public static double ROTARY_KI = 0.000000000;
    public static double ROTARY_KD = 0.000002;
    public static int ROTARY_TICKS_PER_REVOLUTION = 4000;
    public static double ROTARY_THRESHOLD = 0.2;
    public static double purpleColor = 0.7;
    public static double greenColor = 0.5;

    public static final class Colors {
        // Color sensor cut offs for UNKNOWN
        public static double MIN_PROB = 0.001;
        public static double THRESHOLD = 3.0;
        public static boolean USE_MAHALANOBIS_DISTANCE = false;

        public static final class Purple {
            public static final double MEAN_H = 110.83;
            public static final double SIGMA_H = 7.07;
            public static final double MEAN_S = 151.81;
            public static final double SIGMA_S = 13.23;
            public static final double MEAN_V = 128.43;
            public static final double SIGMA_V = 12.00;
        }

        public static final class Green {
            // GREEN BALL
            public static final double MEAN_H = 82.87;
            public static final double SIGMA_H = 6.00;
            public static final double MEAN_S = 185.37;
            public static final double SIGMA_S = 15.33;
            public static final double MEAN_V = 120.73;
            public static final double SIGMA_V = 12.00;
        }
    }

    public static final class HWMap {

        // Drive Motors
        public static final String FL_MOTOR = "front_left_motor";
        public static final String FR_MOTOR = "front_right_motor";
        public static final String BL_MOTOR = "back_left_motor";
        public static final String BR_MOTOR = "back_right_motor";

        // System Motors
        public static final String LAUNCHER_MOTOR = "launcher_motor";
        public static final String INTAKE_MOTOR = "intake_motor";

        // Odometry
        public static final String ODO = "odo";
        public static final String IMU = "imu";

        // Vision
        public static final String LIMELIGHT = "limelight";

        // Ball Selector Hardware
        public static final String ROTARY_SERVO = "selector_servo";
        public static final String PUSH_SERVO = "push_servo";

        // Selector Pos Sensors
        public static final String ELC_ANALOG = "rotary_encoder";
        public static final String ELC_DIGITAL = "elc_digital";

        // Selector Color Sensors
        public static final String COLOR_SENSOR_BOTTOM = "cs_bottom";
        public static final String COLOR_SENSOR_LEFT = "cs_left";
        public static final String COLOR_SENSOR_RIGHT = "cs_right";

        // Light
        public static final String LIGHT = "illuminant_panel";
    }
}