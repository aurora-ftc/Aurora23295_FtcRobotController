package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Constants {
    public static boolean DEBUG = true;
    public static boolean BLUE_SIDE = false;
    public static boolean IS_FIELD_CENTRIC = true;
    public static final int APRIL_TAG_DETECTION = 0;
    public static final double MOSAIC_FLASH_INTERVAL = 0.5;
    public static class Pose {
        public static final Pose2D GOAL_POSE_BLUE = new Pose2D(DistanceUnit.INCH, -63, 62, AngleUnit.DEGREES, 0);
        public static final Pose2D GOAL_POSE_RED = new Pose2D(DistanceUnit.INCH, 63, 62, AngleUnit.DEGREES, 0);
        public static final Pose2D INITIAL_POSE_BLUE = new Pose2D(DistanceUnit.INCH, -13, -35, AngleUnit.DEGREES, 90);
        public static final Pose2D INITIAL_POSE_RED = new Pose2D(DistanceUnit.INCH, 13, -35, AngleUnit.DEGREES, 90);
    }

    public static class PID {
        public static class Drive{
            public static double KP = 1.3;
            public static double KI = 0.0;
            public static double KD = 0.002;
        }
        public static class Flywheel{
            public static double KP = 0.06;
            public static double KI = 0.05;
            public static double KD = 0.0025;
            public static double KV = 0.01;
            public static double KS = 0.0055;

            // Range for Battery Adjusted Kv
            public static double MAX_KV = 0.014;
            public static double MIN_KV = 0.006;
        }
        public static class Rotary{
            public static double ROTARY_KP = 0.000189;
            public static double ROTARY_KI = 0.000;
            public static double ROTARY_KD = 0.0000119;
            public static double ROTARY_KS = 0.055;
        }
    }

    // Ideal Volts
    public static double VOLTS_NOMINAL = 12.5;
    // Launcher Ticks Per Rev
    public static double LAUNCHER_ENCODER_PER_REV = 28.0;
    // Slow Speed Modifier
    public static double SLOW_SPEED_LT = 0.7;
    public static double[] POWER_STEPS = {50, 55, 58, 65, 70, 75, 85};
    public static int[] POSITIONS = {0, 667, 1333, 2000, 2666, 3333};
    public static float revColorSensorGain = 0.4f;

    public static int ROTARY_TICKS_PER_REVOLUTION = 4000;
    public static double ROTARY_THRESHOLD = 0.2;
    public static double purpleColor = 0.7;
    public static double greenColor = 0.5;

    public static final double GATE_OPEN = 0;
    public static final double GATE_CLOSE = 1;

    public static final double CLOSE = 1;
    public static final double OPEN = 0;

    public static double VELOCITY_THRESHOLD = 100;

    public static final class ColorConfig {
        // Color sensor cut offs for UNKNOWN
        public static double MIN_PROB = 0.001;
        public static double THRESHOLD = 2.0;
        public static boolean USE_MAHALANOBIS_DISTANCE = true;

        public static final class Green {
            public static final double MEAN_H = 77.97;
            public static final double SIGMA_H = 6.00;
            public static final double MEAN_S = 183.07;
            public static final double SIGMA_S = 12.00;
            public static final double MEAN_V = 127.27;
            public static final double SIGMA_V = 12.00;
        }

        public static final class Purple {
            // GREEN BALL
            public static final double MEAN_H = 111.35;
            public static final double SIGMA_H = 7.83;
            public static final double MEAN_S = 114.48;
            public static final double SIGMA_S = 12.00;
            public static final double MEAN_V = 114.45;
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
        public static final String ROTARY_SERVO = "selector_motor";
        public static final String INDEXER_MOTOR = "indexer_motor";
        public static final String PUSH_SERVO = "push_servo";
        public static final String GATE_SERVO = "gate_servo";
        public static final String LIFT_SERVO = "lift_servo";


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