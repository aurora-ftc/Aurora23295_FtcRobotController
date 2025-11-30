package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.Constants;

import java.util.Arrays;

public class BallSelector {

    private DcMotorEx selectorMotor;
    private Servo selectorServo;
    private RevColorSensorV3[] colorSensors = new RevColorSensorV3[3];

    // Constants
    // TODO: Adjust these constants accordingly
    private static final double SPIN_POWER = 0.5;
    private static final double SPIN_TIME = 0.5;
    private static final double PUSH_POSITION = 0.9; // Adjust as needed
    private static final double RETRACT_POSITION = 0.5; // Adjust as needed

    public enum BallColor {
        PURPLE, GREEN, UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        selectorMotor = hwMap.get(DcMotorEx.class, Constants.HWMap.SELECTOR_MOTOR);
        selectorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        selectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        selectorServo = hwMap.get(Servo.class, Constants.HWMap.SELECTOR_SERVO);
        selectorServo.setPosition(RETRACT_POSITION);

        colorSensors[0] = hwMap.get(RevColorSensorV3.class, Constants.HWMap.COLOR_SENSOR_1);
        colorSensors[1] = hwMap.get(RevColorSensorV3.class, Constants.HWMap.COLOR_SENSOR_2);
        colorSensors[2] = hwMap.get(RevColorSensorV3.class, Constants.HWMap.COLOR_SENSOR_3);
    }

    public void spin(double power) {
        selectorMotor.setPower(power);
    }

    public void stopSpin() {
        selectorMotor.setPower(0);
    }

    public void pushBall() {
        selectorServo.setPosition(PUSH_POSITION);
    }

    public void resetPusher() {
        selectorServo.setPosition(RETRACT_POSITION);
    }

    public BallColor[] readColors() {
        BallColor[] colors = new BallColor[3];
        for (int i = 0; i < 3; i++) {
            colors[i] = detectColor(colorSensors[i]);
        }
        return colors;
    }

    public Action spinForTime(double power, double duration) {
        return new Action() {
            private boolean init = false;
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init)
                    timer.reset();
                init = true;
                selectorMotor.setPower(power);
                return timer.seconds() < duration;
            }
        };
    }

    public void spinToColor(BallColor target, Telemetry telemetry) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 5000) { // 5 second timeout
            BallColor[] currentColors = readColors();
            // Assuming Sensor 1 (index 0) is the exit sensor
            if (currentColors[0] == target) {
                continue;
            } else if (currentColors[1] == target) {
                // TODO: Need to figure out the direction according to the sensor placement
                spinForTime(SPIN_POWER, SPIN_TIME);
                stopSpin();
            } else if (currentColors[2] == target) {
                spinForTime(-SPIN_POWER, SPIN_TIME);
                stopSpin();
            }
            // telemetry.addData("Target", target);
            // telemetry.addData("Seen", Arrays.toString(currentColors));
            // telemetry.update();
        }
    }

    private BallColor detectColor(RevColorSensorV3 sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r * 8, g * 8, b * 8, hsv); // Scale up RGB for conversion if needed, or just use raw

        // Simple thresholding based on Hue
        // Purple Hue is around 270-300 (or 110-130 if 0-180 scale)
        // Green Hue is around 120 (or 60-90 if 0-180 scale)

        // Using the values from BallColorDetection as reference:
        // Purple Mean H: 110 (0-180 scale) -> ~220 (0-360)
        // Green Mean H: 82 (0-180 scale) -> ~164 (0-360)

        float hue = hsv[0];

        if (hue > 100 && hue < 140) { // Adjust these ranges based on calibration
            return BallColor.GREEN; // 120 is pure green
        } else if (hue > 200 && hue < 300) {
            return BallColor.PURPLE; // 270 is pure purple
        }

        // Fallback or more complex logic can be added here
        return BallColor.UNKNOWN;
    }

    public void telemetry(Telemetry telemetry) {
        BallColor[] colors = readColors();
        telemetry.addData("Ball Selector Colors", Arrays.toString(colors));
        telemetry.addData("Sensor 1 H", getHue(colorSensors[0]));
        telemetry.addData("Sensor 2 H", getHue(colorSensors[1]));
        telemetry.addData("Sensor 3 H", getHue(colorSensors[2]));
    }

    private float getHue(RevColorSensorV3 sensor) {
        float[] hsv = new float[3];
        Color.RGBToHSV(sensor.red() * 8, sensor.green() * 8, sensor.blue() * 8, hsv);
        return hsv[0];
    }
}