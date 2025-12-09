package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.HWMap;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.MOSAIC_FLASH_INTERVAL;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.POSITIONS;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.ROTARY_KD;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.ROTARY_KI;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.ROTARY_KP;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.greenColor;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanH_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanH_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanS_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanS_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanV_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanV_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.minProb;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.purpleColor;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaH_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaH_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaS_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaS_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaV_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaV_purple;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.teamcode.teleOp.util.MathUtils;
import org.firstinspires.ftc.teamcode.teleOp.util.Mosaic;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;

import java.util.Arrays;

public class BallSelector extends SubsystemBase {
    private static final double MOSAIC_WAIT = 1.0;
    // Mosaic flashing variables
    private final ElapsedTime mosaicFlashTimer = new ElapsedTime();
    double[] positions = new double[6];
    float[] hsv = new float[3];
    Colors[] stored;
    double h, s, v;
    double angle;
    double p_purple, p_green;
    CRServo rotaryServo;
    Servo pushServo;
    RevColorSensorV3 colorBottom, colorLeft, colorRight;
    Servo light;
    AnalogInput encoder;
    PIDController controller;
    double time;
    int currentPositionIndex = 0;
    private boolean complete = false;
    private int currentMosaicColorIndex = 0;

    public BallSelector() {

    }

    public void init(HardwareMap map) {
        rotaryServo = map.get(CRServo.class, Constants.HWMap.ROTARY_SERVO);
        pushServo = map.get(Servo.class, HWMap.PUSH_SERVO);
        encoder = map.get(AnalogInput.class, HWMap.ENCODER);
        colorBottom = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_BOTTOM);
        colorLeft = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_LEFT);
        colorRight = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_RIGHT);
        light = map.get(Servo.class, HWMap.LIGHT);

        positions = POSITIONS;

        controller = new PIDController(ROTARY_KP, ROTARY_KI, ROTARY_KD);

        // Initialize to first position
        currentPositionIndex = 0;
        controller.setTarget(positions[0]);
    }

    public void periodic() {
        time = System.nanoTime() / 1e9;
        rotaryServo.setPower(controller.calculateOutputPID(angle,
                time, true));
        angle = (encoder.getVoltage() / 3.3) / 360;
    }

    public void setTargetPosition(int position) {
        currentPositionIndex = position;
        controller.setTarget(positions[position]);
    }

    public void moveUp() {
        currentPositionIndex = (currentPositionIndex + 1) % positions.length;
        controller.setTarget(positions[currentPositionIndex]);
    }

    public void moveDown() {
        currentPositionIndex = (currentPositionIndex - 1 + positions.length) % positions.length;
        controller.setTarget(positions[currentPositionIndex]);
    }

    public boolean isAtTarget() {
        return Math.abs(angle - controller.getTarget()) < 0.01;
    }

    public Colors getColor(RevColorSensorV3 sensor) {
        if (sensor == null) {
            return Colors.Unknown;
        }

        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();

        double sum = r + g + b;

        double normalRed = r / sum * 255;
        double normalGreen = g / sum * 255;
        double normalBlue = b / sum * 255;

        android.graphics.Color.RGBToHSV((int) normalRed, (int) normalGreen, (int) normalBlue, hsv);
        h = hsv[0] / 2.0; // convert 0–360 → 0–180
        s = hsv[1] * 255.0;
        v = hsv[2] * 255.0;

        p_purple = MathUtils.gaussian3D(h, s, v,
                meanH_purple, sigmaH_purple,
                meanS_purple, sigmaS_purple,
                meanV_purple, sigmaV_purple);

        p_green = MathUtils.gaussian3D(h, s, v,
                meanH_green, sigmaH_green,
                meanS_green, sigmaS_green,
                meanV_green, sigmaV_green);

        if (p_purple > p_green && p_purple > minProb) {
            return Colors.Purple;
        } else if (p_purple < p_green && p_green > minProb) {
            return Colors.Green;
        } else {
            return Colors.Unknown;
        }
    }

    public void getColour() {
        stored = new Colors[]{getColor(colorBottom), getColor(colorLeft), getColor(colorRight)};
    }

    public void loadBall() {
        getColour();
        if (Arrays.asList(stored).contains(Colors.Unknown)) {
            setTargetPosition(Arrays.asList(stored).indexOf(Colors.Unknown));
        }
    }

    public void returnBall(Colors color) {
        getColour();
        if (Arrays.asList(stored).contains(color)) {
            setTargetPosition(Arrays.asList(stored).indexOf(color) + 3);
        }
    }

    public void push() {
        pushServo.setPosition(1);
        pushServo.setPosition(0);
    }

    /**
     * Flashes the mosaic pattern periodically on the indicator light.
     * The pattern cycles through the colors: Purple-Purple-Green, Green-Purple-Purple, or Purple-Green-Purple
     * based on the detected mosaic pattern.
     */
    public void flashMosaicPattern() {
        Mosaic mosaic = Storage.mosaic;

        // If mosaic is unknown, don't flash
        if (mosaic == null || mosaic == Mosaic.UNKNOWN) {
            return;
        }
        if (mosaicFlashTimer.seconds() < MOSAIC_WAIT && !complete) {
            // Check if it's time to advance to the next color
            if (mosaicFlashTimer.seconds() >= MOSAIC_FLASH_INTERVAL) {
                String mosaicString = mosaic.toString();

                // Get the current color character (P or G)
                char currentColorChar = mosaicString.charAt(currentMosaicColorIndex);

                // Set the light color based on the character
                if (currentColorChar == 'p') {
                    // Set to purple color
                    light.setPosition(purpleColor);
                } else {
                    // Set to green color
                    light.setPosition(greenColor);
                }

                // Move to next color in the sequence
                if (currentMosaicColorIndex == mosaicString.length()) {
                    complete = true;
                }
                currentMosaicColorIndex = (currentMosaicColorIndex + 1) % mosaicString.length();

                // Reset timer for next flash
                mosaicFlashTimer.reset();
            }
        } else if (mosaicFlashTimer.seconds() > MOSAIC_WAIT) {
            complete = false;
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        multiTelemetry.addLine("===== Ball Selector Telemetry =====");
        multiTelemetry.addData("Array stored", stored);
        multiTelemetry.addData("Current Position", positions);
        multiTelemetry.addData("Current Angle", angle);
        multiTelemetry.addData("Target Angle", controller.getTarget());
        multiTelemetry.addData("At Target", isAtTarget());

        multiTelemetry.update();
    }
}

