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
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.teamcode.teleOp.util.MathUtils;
import org.firstinspires.ftc.teamcode.teleOp.util.Mosaic;

import java.util.Arrays;

public class BallSelector extends SubsystemBase {
    private static final double MOSAIC_WAIT = 1.0;
    // Mosaic flashing variables
    private final ElapsedTime mosaicFlashTimer = new ElapsedTime();
    boolean canMove = true;
    int[] positions = new int[6];
    float[] hsv = new float[3];
    Colors[] stored;
    double h, s, v;
    double angle;
    double p_purple, p_green;
    CRServo rotaryServo;
    Servo pushServo;
    double pushTime = 0.0;
    RevColorSensorV3 colorBottom, colorLeft, colorRight;
    Servo light;
    AnalogInput encoder;
    DcMotor dcEncoder;
    PIDController controller;
    double time;
    double lightColor = greenColor;
    private int currentPositionIndex = 0;
    private boolean complete = false;
    private int currentMosaicColorIndex = 0;
    private int target = 0;

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

        pushServo.scaleRange(0.0, 0.25);

        // Initialize to first position
        currentPositionIndex = 0;
        pushServo.setPosition(0);
        dcEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        rotaryServo.setDirection(DcMotorSimple.Direction.FORWARD);

        dcEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Periodic: regular update for the indexer
     *
     * @author James Beers
     */

    public void periodic() {
        controller.setPID(ROTARY_KP, ROTARY_KI, ROTARY_KD);

        time = System.nanoTime();

        double output = controller.calculate(dcEncoder.getCurrentPosition(), target);
        int sign = 1;
        if (output < 0) {
            sign = -1;
            output *= -1;
        }
        output = Math.sqrt(output);
        output *= sign;
//
        rotaryServo.setPower(Range.clip(output, -1, 1));

        if (time - pushTime >= PUSH_SERVO_FLICK_TIME) {
            pushServo.setPosition(0);
        }

        getColour();
    }

    /**
     * setTargetPosition: set the position of the indexer
     *
     * @param position the target position according to POSITIONS
     * @author James Beers
     */
    public void setTargetPosition(int position) {
        if (canMove) {
            target = position;
        }
    }

    /**
     * moveUp and moveDown: for use with d-Pad controls.
     *
     * @author James Beers
     */
    public void moveUp() {
        currentPositionIndex = (currentPositionIndex + 1) % positions.length;
        target = positions[currentPositionIndex];
    }

    public void moveDown() {
        currentPositionIndex = (currentPositionIndex - 1 + positions.length) % positions.length;
        target = positions[currentPositionIndex];
    }

    /**
     * isAtTarget: check if the system is close to the target value.
     *
     * @return <b><i>TRUE</i></b> if the indexer is close enough to the target
     * @author James Beers
     */
    public boolean isAtTarget() {
        return Math.abs(dcEncoder.getCurrentPosition() - target) < ROTARY_THRESHOLD;
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
        if (Arrays.asList(stored).contains(Colors.Unknown) && getColor(colorBottom) != Colors.Unknown) {
            setTargetPosition(Arrays.asList(stored).indexOf(Colors.Unknown));
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
        if (isAtTarget()) {
            pushServo.setPosition(1);
            pushTime = time;
        }
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
        multiTelemetry.addData("Target Angle", target);
        multiTelemetry.addData("Servo Power", rotaryServo.getPower());
        multiTelemetry.addData("At Target", isAtTarget());
        multiTelemetry.addLine();
        multiTelemetry.update();
    }
}

