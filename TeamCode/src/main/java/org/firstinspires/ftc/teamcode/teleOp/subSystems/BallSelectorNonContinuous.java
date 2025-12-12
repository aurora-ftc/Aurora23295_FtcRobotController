package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.HWMap;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.MOSAIC_FLASH_INTERVAL;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.POSITIONS;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.teleOp.util.MathUtils;
import org.firstinspires.ftc.teamcode.teleOp.util.Mosaic;

import java.util.Arrays;

public class BallSelectorNonContinuous extends SubsystemBase {
    private static final double MOSAIC_WAIT = 1.0;
    // Mosaic flashing variables
    private final ElapsedTime mosaicFlashTimer = new ElapsedTime();
    double[] positions = new double[6];
    float[] hsv = new float[3];
    Colors[] stored;
    double h, s, v;
    double angle;
    double p_purple, p_green;
    Servo rotaryServo;
    Servo pushServo;
    RevColorSensorV3 colorBottom, colorLeft, colorRight;
    Servo light;
    double time;
    int currentPositionIndex = 0;
    double lightColor = greenColor;
    private boolean complete = false;
    private int currentMosaicColorIndex = 0;

    public BallSelectorNonContinuous() {

    }

    /**
     * Init: initialize all hardware and PID
     *
     * @param map hardware map object
     * @author James Beers
     */

    public void init(HardwareMap map) {
        positions = POSITIONS;

        rotaryServo = map.get(Servo.class, HWMap.ROTARY_SERVO);
        pushServo = map.get(Servo.class, HWMap.PUSH_SERVO);
        colorBottom = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_BOTTOM);
        colorLeft = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_LEFT);
        colorRight = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_RIGHT);
        light = map.get(Servo.class, HWMap.LIGHT);

        // Initialize to first position
        currentPositionIndex = 0;
        rotaryServo.resetDeviceConfigurationForOpMode();
    }

    /**
     * Periodic: regular update for the indexer
     *
     * @author James Beers
     */

    public void periodic() {
        rotaryServo.setPosition(currentPositionIndex);
    }

    public void moveUp() {
        currentPositionIndex = Math.min(positions.length - 1, currentPositionIndex + 1);
    }

    public void moveDown() {
        currentPositionIndex = Math.max(0, currentPositionIndex - 1);
    }

    /**
     * getColor: gets the color read by a color sensor using gaussian probability.
     *
     * @param sensor the color sensor to read
     * @return color most likely detected
     * @author James Beers
     */
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

    /**
     * getColour: write the values to the <b>stored</b> array
     *
     * @author James Beers
     */
    public void getColour() {
        stored = new Colors[]{getColor(colorBottom), getColor(colorLeft), getColor(colorRight)};
    }

    /**
     * loadBall: sets the indexer to the intake position of a (probably) empty tray
     *
     * @author James Beers
     */
    public void loadBall() {
        getColour();
        if (Arrays.asList(stored).contains(Colors.Unknown)) {
            rotaryServo.setPosition(positions[Arrays.asList(stored).indexOf(Colors.Unknown)]);
        }
    }

    /**
     * returnBall: sets the indexer to the position of a given color
     *
     * @param color color to search for
     * @author James Beers
     */
    public void returnBall(Colors color) {
        getColour();
        if (Arrays.asList(stored).contains(color)) {
            rotaryServo.setPosition(positions[Arrays.asList(stored).indexOf(color)]);
        }
    }

    /**
     * push: actuate the push servo
     *
     * @author James Beers
     */
    public void push() {
        pushServo.setPosition(1);
        pushServo.setPosition(0);
    }

    public void lampOn() {
        light.setPosition(lightColor);
    }


    /**
     * flashMosaicPattern: flashes the mosaic pattern periodically on the indicator light.
     * The pattern cycles through the colors: Purple-Purple-Green, Green-Purple-Purple, or Purple-Green-Purple
     * based on the detected mosaic pattern.
     * Doesn't flash if the mosaic is unknown
     *
     * @author James Beers
     */
    public void flashMosaicPattern() {
        Mosaic mosaic = Storage.mosaic;

        if (mosaic == null || mosaic == Mosaic.UNKNOWN) {
            return;
        }
        if (mosaicFlashTimer.seconds() < MOSAIC_WAIT && !complete) {
            if (mosaicFlashTimer.seconds() >= MOSAIC_FLASH_INTERVAL) {
                String mosaicString = mosaic.toString();

                char currentColorChar = mosaicString.charAt(currentMosaicColorIndex);

                if (currentColorChar == 'p') {
                    light.setPosition(purpleColor);
                } else {
                    light.setPosition(greenColor);
                }

                if (currentMosaicColorIndex == mosaicString.length()) {
                    complete = true;
                }
                currentMosaicColorIndex = (currentMosaicColorIndex + 1) % mosaicString.length();

                mosaicFlashTimer.reset();
            }
        } else if (mosaicFlashTimer.seconds() > MOSAIC_WAIT) {
            complete = false;
        }
    }

    /**
     * updateTelemetry: standard telemetry update
     *
     * @param telemetry telemetry object for the driver station
     * @author James Beers
     */
    public void updateTelemetry(Telemetry telemetry) {
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        multiTelemetry.addLine("==()== Ball Selector Telemetry ==()==");
        multiTelemetry.addData("Array stored", " bottom: " + stored[0] + ", left: " + stored[1] + ", right: " + stored[2]);
        multiTelemetry.addData("Current Position Index", currentPositionIndex);
        multiTelemetry.addData("Current Target:", positions[currentPositionIndex]);
        multiTelemetry.addData("Current Position", rotaryServo.getPosition());
        multiTelemetry.update();
    }
}

