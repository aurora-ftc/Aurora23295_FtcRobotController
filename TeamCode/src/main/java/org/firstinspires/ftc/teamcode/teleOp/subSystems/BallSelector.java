package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    DcMotor dcEncoder;
    PIDController controller;
    double time;
    int currentPositionIndex = 0;
    double lightColor = greenColor;
    private boolean complete = false;
    private int currentMosaicColorIndex = 0;

    public BallSelector() {

    }

    /**
     * Init: initialize all hardware and PID
     *
     * @param map hardware map object
     * @author James Beers
     */

    public void init(HardwareMap map) {
        positions = POSITIONS;

        rotaryServo = map.get(CRServo.class, HWMap.ROTARY_SERVO);
        pushServo = map.get(Servo.class, HWMap.PUSH_SERVO);
        encoder = map.get(AnalogInput.class, HWMap.ELC_ANALOG);
        dcEncoder = map.get(DcMotor.class, HWMap.ELC_DIGITAL);
        colorBottom = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_BOTTOM);
        colorLeft = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_LEFT);
        colorRight = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_RIGHT);
        light = map.get(Servo.class, HWMap.LIGHT);

        colorBottom.setGain(revColorSensorGain);
        colorRight.setGain(revColorSensorGain);
        colorLeft.setGain(revColorSensorGain);

        controller = new PIDController(ROTARY_KP, ROTARY_KI, ROTARY_KD);

        // Initialize to first position
        currentPositionIndex = 0;
        controller.setTarget(positions[0]);

        dcEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        rotaryServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Periodic: regular update for the indexer
     *
     * @author James Beers
     */

    public void periodic() {
        time = System.nanoTime();
        angle = ((double) dcEncoder.getCurrentPosition() / ROTARY_TICKS_PER_REVOLUTION * 360 % 360);
        rotaryServo.setPower(
                controller.calculateOutputPID(
                        Math.toRadians(angle), time, true));

        getColour();
    }

    /**
     * setTargetPosition: set the position of the indexer
     *
     * @param position the target position according to POSITIONS
     * @author James Beers
     */
    public void setTargetPosition(int position) {
        currentPositionIndex = position;
        controller.setTarget(positions[position]);
    }

    /**
     * moveUp and moveDown: for use with d-Pad controls.
     *
     * @author James Beers
     */
    public void moveUp() {
        currentPositionIndex = (currentPositionIndex + 1) % positions.length;
        controller.setTarget(positions[currentPositionIndex]);
    }

    public void moveDown() {
        currentPositionIndex = (currentPositionIndex - 1 + positions.length) % positions.length;
        controller.setTarget(positions[currentPositionIndex]);
    }

    /**
     * isAtTarget: check if the system is close to the target value.
     *
     * @return <b><i>TRUE</i></b> if the indexer is close enough to the target
     * @author James Beers
     */
    public boolean isAtTarget() {
        return Math.abs(angle - controller.getTarget()) < ROTARY_THRESHOLD;
    }

    /**
     * getColour: write the values to the <b>stored</b> array
     *
     * @author James Beers
     */
    public void getColour() {
        stored = new Colors[]{Colors.getColor(colorBottom), Colors.getColor(colorLeft), Colors.getColor(colorRight)};
    }

    /**
     * loadBall: sets the indexer to the intake position of a (probably) empty tray
     *
     * @author James Beers
     */
    public void loadBall() {
        if (Arrays.asList(stored).contains(Colors.UNKNOWN)) {
            setTargetPosition(Arrays.asList(stored).indexOf(Colors.UNKNOWN));
        }
    }

    /**
     * returnBall: sets the indexer to the position of a given color
     *
     * @param color color to search for
     * @author James Beers
     */
    public void returnBall(Colors color) {
        if (Arrays.asList(stored).contains(color)) {
            setTargetPosition(Arrays.asList(stored).indexOf(color) + 3);
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
        multiTelemetry.addData("Current Position", positions);
        multiTelemetry.addData("DC Encoder data", dcEncoder.getCurrentPosition());
        multiTelemetry.addData("Current Angle (RAD)", Math.toRadians(angle));
        multiTelemetry.addData("Target Angle", controller.getTarget());
        multiTelemetry.addData("Servo Power", MathUtils.sigmoid(controller.calculateOutputPID(
                Math.toRadians(angle), time, true)));
        multiTelemetry.addData("At Target", isAtTarget());
        multiTelemetry.addLine();
        multiTelemetry.addData("PID constants", controller.kp + ", " + controller.ki + ", " + controller.kd);

        multiTelemetry.update();
    }
}

