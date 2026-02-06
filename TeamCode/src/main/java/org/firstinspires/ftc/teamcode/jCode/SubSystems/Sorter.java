package org.firstinspires.ftc.teamcode.jCode.SubSystems;

import static org.firstinspires.ftc.teamcode.jCode.Configuration.HWMap;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.SORT_KD;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.SORT_KI;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.SORT_KP;
import static org.firstinspires.ftc.teamcode.jCode.Enumerators.Artifacts.Green;
import static org.firstinspires.ftc.teamcode.jCode.Enumerators.Artifacts.Purple;
import static org.firstinspires.ftc.teamcode.jCode.Enumerators.Artifacts.Unknown;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.ColorConfig.MIN_PROB;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.jCode.Enumerators.Artifacts;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.teamcode.teleOp.util.MathUtils;

import java.util.Arrays;

/**<p>Before you change anything...</p>
 * <b>DID YOU READ MY README? </b>
 * <p>thank you for your understanding</p>
 */

public class Sorter {
    private PIDController controller;
    private DcMotor rotor;
    private RevColorSensorV3 colorSensorBottom, colorSensorRight, colorSensorLeft;
    private boolean outtake = false, rotateUp = false;
    private final Artifacts[] stored = {Unknown, Unknown, Unknown};
    private Artifacts temp = Unknown;
    private int pos = 0;

    public void init(HardwareMap map) {
        controller = new PIDController(SORT_KP, SORT_KI, SORT_KD);
        rotor = map.get(DcMotor.class, HWMap.rotaryActuator);
        controller.setSetPoint(0.0);
        colorSensorRight = map.get(RevColorSensorV3.class, HWMap.rColorSensor);
        colorSensorBottom = map.get(RevColorSensorV3.class, HWMap.bColorSensor);
        colorSensorLeft = map.get(RevColorSensorV3.class, HWMap.lColorSensor);
    }

    public void rotateUp() {
        pos += 1333;
        rotateUp = true;
        temp = stored[2];
        stored[2] = stored[1];
        stored[1] = stored[0];
        stored[0] = temp;
    }

    public void rotateDown() {
        pos -= 1333;
        rotateUp = false;
        temp = stored[0];
        stored[0] = stored[1];
        stored[1] = stored[2];
        stored[2] = temp;
    }

    public void switchState() {
        outtake = !outtake;
        pos = rotateUp ? pos - 667 : pos + 667;
    }

    public void setState() {
        if (!outtake) {
            switchState();
        }
    }

    public void periodic() {
        rotor.setPower(controller.calculate(pos));
        if (colorSensorBottom.getDistance(DistanceUnit.MM) > 5 && !outtake) {
            stored[0] = getColor(colorSensorBottom);
            if (stored[1] == Unknown) {
                rotateUp();
            } else if (stored[2] == Unknown) {
                rotateDown();
            }
        }
        if (!outtake && !Arrays.asList(stored).contains(Unknown)) {
            switchState();
        }
    }

    public boolean atTarget() {
        return controller.atSetPoint();
    }

    public Artifacts getColor(RevColorSensorV3 sensor) {
        if (sensor == null) return Unknown;
        float[] hsv = new float[3];

        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();

        double sum = r + g + b;
        if (sum == 0) sum = 1;

        double normalRed = r / sum * 255;
        double normalGreen = g / sum * 255;
        double normalBlue = b / sum * 255;

        android.graphics.Color.RGBToHSV((int) normalRed, (int) normalGreen, (int) normalBlue, hsv);
        double h = hsv[0] / 2.0; // convert 0–360 → 0–180
        double s = hsv[1] * 255.0;
        double v = hsv[2] * 255.0;

        double p_purple, p_green;

        Artifacts detectedArtifact;

        p_purple = MathUtils.gaussian3D(h, s, v, Constants.ColorConfig.Purple.MEAN_H, Constants.ColorConfig.Purple.SIGMA_H, Constants.ColorConfig.Purple.MEAN_S, Constants.ColorConfig.Purple.SIGMA_S, Constants.ColorConfig.Purple.MEAN_V, Constants.ColorConfig.Purple.SIGMA_V);

        p_green = MathUtils.gaussian3D(h, s, v, Constants.ColorConfig.Green.MEAN_H, Constants.ColorConfig.Green.SIGMA_H, Constants.ColorConfig.Green.MEAN_S, Constants.ColorConfig.Green.SIGMA_S, Constants.ColorConfig.Green.MEAN_V, Constants.ColorConfig.Green.SIGMA_V);

        if (p_purple > p_green && p_purple < MIN_PROB) {
            detectedArtifact = Purple;
        } else if (p_green > p_purple && p_green > MIN_PROB) {
            detectedArtifact = Green;
        } else {
            detectedArtifact = Unknown;
        }

        return detectedArtifact;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Sorter Controller");
        telemetry.addData("Sorter status", "OK");
        telemetry.addData("Active Status", "OK");
    }
}
