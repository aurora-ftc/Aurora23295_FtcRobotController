package org.firstinspires.ftc.teamcode.teleOp.util;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.Colors.Green;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.Colors.MIN_PROB;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.Colors.Purple;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.Colors.THRESHOLD;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.Colors.USE_MAHALANOBIS_DISTANCE;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public enum Colors {
    GREEN,
    PURPLE,
    UNKNOWN;

    public static Colors getColor(RevColorSensorV3 sensor) {
        if (sensor == null) return Colors.UNKNOWN;

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
        double d_purple, d_green;

        Colors detectedColor;

        if (USE_MAHALANOBIS_DISTANCE) {

            d_purple = MathUtils.mahalanobisDistance(h, s, v,
                    Purple.MEAN_H, Purple.SIGMA_H,
                    Purple.MEAN_S, Purple.SIGMA_S,
                    Purple.MEAN_V, Purple.SIGMA_V);

            d_green = MathUtils.mahalanobisDistance(h, s, v,
                    Green.MEAN_H, Green.SIGMA_H,
                    Green.MEAN_S, Green.SIGMA_S,
                    Green.MEAN_V, Green.SIGMA_V);

            if (d_purple < THRESHOLD && d_purple < d_green)
                detectedColor = PURPLE;
            else if (d_green < THRESHOLD && d_green < d_purple)
                detectedColor = GREEN;
            else
                detectedColor = UNKNOWN;

        } else {

            p_purple = MathUtils.gaussian3D(h, s, v,
                    Purple.MEAN_H, Purple.SIGMA_H,
                    Purple.MEAN_S, Purple.SIGMA_S,
                    Purple.MEAN_V, Purple.SIGMA_V);

            p_green = MathUtils.gaussian3D(h, s, v,
                    Green.MEAN_H, Green.SIGMA_H,
                    Green.MEAN_S, Green.SIGMA_S,
                    Green.MEAN_V, Green.SIGMA_V);

            if (p_purple > p_green && p_purple < MIN_PROB)
                detectedColor = PURPLE;
            else if (p_green > p_purple && p_green > MIN_PROB)
                detectedColor = GREEN;
            else
                detectedColor = UNKNOWN;
        }

        return detectedColor;

    }
}
