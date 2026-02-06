package org.firstinspires.ftc.teamcode.jCode;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;

/**<p>Before you change anything...</p>
 * <b>DID YOU READ MY README? </b>
 * <p>thank you for your understanding</p>
 */


@Configurable
public class Configuration {
    public static double DRIVE_KP = 2.3;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.003;
    public static double SORT_KP = 0.00028;
    public static double SORT_KI = 0.000;
    public static double SORT_KD = 0.000017;
    public static double FLYWHEEL_KP = 0.08;
    public static double FLYWHEEL_KI = 0.05;
    public static double FLYWHEEL_KD = 0.05;
    public static double FLYWHEEL_KF = 0.0;
    public static class HWMap {
        public static final String flMotor = "front_left_motor";
        public static final String frMotor = "front_right_motor";
        public static final String blMotor = "back_left_motor";
        public static final String brMotor = "back_right_motor";
        public static final String intakeMotor = "intake_motor";
        public static final String outtakeMotor = "launcher_motor";
        public static final String pushServo = "push_servo";
        public static final String odo = "odo";
        public static final String rotaryActuator = "indexer_motor";
        public static final String bColorSensor = "cs_bottom";
        public static final String lColorSensor = "cs_left";
        public static final String rColorSensor = "cs_right";
        public static final String rotaryEncoder = "rotary_encoder";
        public static final String LIGHT = "illuminant_panel";
        public static final String GATE_SERVO = "gate_servo";
    }
}
