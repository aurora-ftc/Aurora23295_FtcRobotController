package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Constants {
    /**
     * Positions for goals and starts
     */
    public static final Pose2D goalPoseBlue = new Pose2D(DistanceUnit.INCH, -63, 62, AngleUnit.DEGREES, 0);
    public static final Pose2D goalPoseRed = new Pose2D(DistanceUnit.INCH, 63, 62, AngleUnit.DEGREES, 0);
    public static final Pose2D initialPoseBlue = new Pose2D(DistanceUnit.INCH, -13, -35, AngleUnit.DEGREES, 90);
    public static final Pose2D initialPoseRed = new Pose2D(DistanceUnit.INCH, 13, -35, AngleUnit.DEGREES, 90);
    public static final int APRIL_TAG_DETECTION = 0;
    public static final double MOSAIC_FLASH_INTERVAL = 0.5;
    /**
     * Mode switches
     */
    public static boolean DEBUG = false;
    public static boolean BLUE_SIDE = false;
    public static boolean IS_FIELD_CENTRIC = true;
    public static double SLOW_SPEED_LT = 0.35; // Slow speed
    /**
     * PID constants
     */
    public static double DRIVE_KP = 2.3;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.003;
    public static double FLYWHEEL_KP = 0.08;   // 0.08
    public static double FLYWHEEL_KI = 0.05;   // 0.05
    public static double FLYWHEEL_KD = 0.002;  // 0.002
    public static double FLYWHEEL_KV = 0.01;   // 0.01
    public static double FLYWHEEL_KS = 0.0055; // 0.004
    public static double MAX_FLYWHEEL_KV = 0.014;
    public static double MIN_FLYWHEEL_KV = 0.006;
    public static double VOLTS_NOMINAL = 12.5;
    public static double LAUNCHER_ENCODER_PER_REV = 28.0;
    /**
     * Lift Servo controls (Zero bot) [Removed] <p>
     * Configuration for color detection system
     */
    public static boolean useMahalanobisDistance = false;
    public static double minProb = 0.001;
    public static double threshold = 3.0;
    // PURPLE BALL
    public static double meanH_purple = 110.83;
    public static double sigmaH_purple = 7.07;
    public static double meanS_purple = 151.81;
    public static double sigmaS_purple = 13.23;
    public static double meanV_purple = 128.43;
    public static double sigmaV_purple = 12.00;
    // GREEN BALL
    public static double meanH_green = 82.87;
    public static double sigmaH_green = 6.00;
    public static double meanS_green = 185.37;
    public static double sigmaS_green = 15.33;
    public static double meanV_green = 120.73;
    public static double sigmaV_green = 12.00;
    /**
     * Launch System configuration
     */
    public static double LIFT_SERVO_FLICK_TIME = 120.0;
    public static double[] POWER_STEPS = {40, 42, 44, 85};
    public static double[] POSITIONS = {0, (double) 1 / 3, (double) 2 / 3, (double) 1 / 6, 0.5, 1};
    public static float revColorSensorGain = 0.4f;
    /**
     * Rotary Selector System Configuration
     */
    public static double ROTARY_KP = 0.1;
    public static double ROTARY_KI = 0.095;
    public static double ROTARY_KD = 0.003;
    public static int ROTARY_TICKS_PER_REVOLUTION = 4000;
    public static double ROTARY_THRESHOLD = 0.2;
    public static double purpleColor = 0.7;
    public static double greenColor = 0.5;

    public static class HWMap {
        public static final String LIGHT = "illuminant_panel";
        public static final String DC_ENCODER = "elc_digital";
        public static String FL_MOTOR = "front_left_motor";
        public static String FR_MOTOR = "front_right_motor";
        public static String BL_MOTOR = "back_left_motor";
        public static String BR_MOTOR = "back_right_motor";

        public static String LAUNCHER_MOTOR = "launcher_motor";
        public static String INTAKE_MOTOR = "intake_motor";

        public static String ODO = "odo";
        public static String IMU = "imu";

        public static String LIMELIGHT = "limelight";

        // Ball Selector Hardware
        public static String ROTARY_SERVO = "selector_servo";
        public static String PUSH_SERVO = "push_servo";
        public static String ENCODER = "rotary_encoder";
        public static String COLOR_SENSOR_BOTTOM = "cs_bottom";
        public static String COLOR_SENSOR_LEFT = "cs_left";
        public static String COLOR_SENSOR_RIGHT = "cs_right";
    }
}