package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.HWMap;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.POSITIONS;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanH_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanH_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanS_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanS_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanV_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.meanV_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.minProb;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaH_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaH_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaS_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaS_purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaV_green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.sigmaV_purple;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.util.MathUtils;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;

import java.util.Arrays;

public class BallSelector {
    double[] positions = new double[6];
    float[] hsv = new float[3];
    Colors[] stored;
    double h, s, v;
    double p_purple, p_green;


    CRServo rotaryServo;
    Servo pushServo;
    RevColorSensorV3 colorBottom, colorLeft, colorRight;
    Encoder encoder;
    PIDController controller;
    double time;

    public BallSelector() {

    }

    public void init(HardwareMap map) {
        rotaryServo = map.get(CRServo.class, HWMap.ROTARY_SERVO);
        pushServo = map.get(Servo.class, HWMap.PUSH_SERVO);
        encoder = map.get(Encoder.class, HWMap.ENCODER);
        colorBottom = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_BOTTOM);
        colorLeft = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_LEFT);
        colorRight = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_RIGHT);

        positions = POSITIONS;

        controller = new PIDController(0.0, 0.0, 0.0);
    }

    public void runToPosition(int position) {
        time = System.nanoTime() / 1e9;
        controller.setTarget(positions[position]);
        rotaryServo.setPower(controller.calculateOutputPID(encoder.getPositionAndVelocity().position,
                time, true));
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

        if (p_purple > p_green && p_purple < minProb) {
            return Colors.Purple;
        } else if (p_purple < p_green && p_green < minProb) {
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
            runToPosition(Arrays.asList(stored).indexOf(Colors.Unknown));
        }
    }

    public void returnBall(Colors color) {
        getColour();
        if (Arrays.asList(stored).contains(color)) {
            runToPosition(Arrays.asList(stored).indexOf(color) + 3);
        }
    }

    public void push() {
        pushServo.setPosition(1);
        pushServo.setPosition(0);
    }

    public void output() {
    }

    public void outputInOrder(Mosaic mosaic) {
        String mos = mosaic.toString();

        if (Arrays.asList(stored).contains(Colors.Unknown)) {
            return;
        }

        for (Character c : mos.toCharArray()) {
            if (c.equals('p')) {
                returnBall(Colors.Purple);
            } else {
                returnBall(Colors.Green);
            }
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Array stored", stored);
        telemetry.addData("Current Position", positions);
    }
}