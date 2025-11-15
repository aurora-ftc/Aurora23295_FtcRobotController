package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BallColorDetection {

    // --- Gaussian color model parameters (HSV) ---
    // These are approximate — tune with telemetry for your exact lighting setup
    // Hue range: 0–180 (since we divide hue/2 like OpenCV)

    private float[] hsv = new float[3];

    private final double minProb = 0.001;
    private final double threshold = 3.0;

    // PURPLE BALL
    private final double meanH_purple = 110.83;
    private final double sigmaH_purple = 7.07;
    private final double meanS_purple = 151.81;
    private final double sigmaS_purple = 13.23;
    private final double meanV_purple = 128.43;
    private final double sigmaV_purple = 12.00;

    // GREEN BALL
    private final double meanH_green = 82.87;
    private final double sigmaH_green = 6.00;
    private final double meanS_green = 185.37;
    private final double sigmaS_green = 15.33;
    private final double meanV_green = 120.73;
    private final double sigmaV_green = 12.00;

    // Sensor instance
    private RevColorSensorV3 colorSensor;

    // --- for telemetry use ---
    private double h, s, v;
    private int r, g, b;
    private double p_purple, p_green;
    private double dPurple, dGreen;
    private DetectedColor detectedColor = DetectedColor.UNKNOWN;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = (RevColorSensorV3) hwMap.get(ColorSensor.class, "color_sensor");
        colorSensor.setGain(10);
    }

    public DetectedColor getDetectedColor() {
        if (colorSensor == null) return DetectedColor.UNKNOWN;

        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();

        // Normalize RGB to reduce lighting effects
        double sum = r + g + b;
        if (sum == 0) sum = 1;
        double rN = r / sum * 255.0;
        double gN = g / sum * 255.0;
        double bN = b / sum * 255.0;

        // Convert to HSV
        android.graphics.Color.RGBToHSV((int) rN, (int) gN, (int) bN, hsv);
        h = hsv[0] / 2.0; // convert 0–360 → 0–180
        s = hsv[1] * 255.0;
        v = hsv[2] * 255.0;

        // Compute Gaussian probabilities
        p_purple = gaussian3D(h, s, v,
                meanH_purple, sigmaH_purple,
                meanS_purple, sigmaS_purple,
                meanV_purple, sigmaV_purple);

        p_green = gaussian3D(h, s, v,
                meanH_green, sigmaH_green,
                meanS_green, sigmaS_green,
                meanV_green, sigmaV_green);

//        // Choose which color is most likely
//        if (p_purple > p_green && p_purple < minProb) {
//            detectedColor = DetectedColor.PURPLE;
//        } else if (p_green > p_purple && p_green > minProb) {
//            detectedColor = DetectedColor.GREEN;
//        } else {
//            detectedColor = DetectedColor.UNKNOWN;
//        }
        dPurple = mahalanobisDistance(h, s, v,
                meanH_purple, sigmaH_purple,
                meanS_purple, sigmaS_purple,
                meanV_purple, sigmaV_purple);

        dGreen = mahalanobisDistance(h, s, v,
                meanH_green, sigmaH_green,
                meanS_green, sigmaS_green,
                meanV_green, sigmaV_green);

        // Choose detected color
        if (dPurple < threshold && dPurple < dGreen) {
            detectedColor = DetectedColor.PURPLE;
        } else if (dGreen < threshold && dGreen < dPurple) {
            detectedColor = DetectedColor.GREEN;
        } else {
            detectedColor = DetectedColor.UNKNOWN;
        }

        return detectedColor;
    }

    public void debugTelemetry(Telemetry tele) {
        tele.addData("Raw RGB", "(%d, %d, %d)", r, g, b);
        tele.addData("HSV", "(%.1f, %.1f, %.1f)", h, s, v);
        tele.addLine();
        tele.addData("Mahalanobis Purple Distance", "%.6f", dPurple);
        tele.addData("Mahalanobis Green Distance", "%.6f", dGreen);
        tele.addLine();
        tele.addData("Gaussian Purple Probability", "%.6f", p_purple);
        tele.addData("Gaussian Green Probability", "%.6f", p_green);
        tele.addData("Detected Color", detectedColor);
        tele.addLine();
        tele.addData("Measured Distance (IN)", colorSensor.getDistance(DistanceUnit.INCH));
    }

    public void compTelemetry(Telemetry tele) {
        tele.addData("Detected Color", detectedColor);
    }

    private double gaussian(double x, double mean, double sigma) {
        return Math.exp(-0.5 * Math.pow((x - mean) / sigma, 2));
    }

    private double gaussian3D(double h, double s, double v,
                              double meanH, double sigmaH,
                              double meanS, double sigmaS,
                              double meanV, double sigmaV) {
        double pH = gaussian(h, meanH, sigmaH);
        double pS = gaussian(s, meanS, sigmaS);
        double pV = gaussian(v, meanV, sigmaV);
        return pH * pS * pV;
    }

    private double mahalanobisDistance(double h, double s, double v,
                                       double meanH, double sigmaH,
                                       double meanS, double sigmaS,
                                       double meanV, double sigmaV) {
        double dh = (h - meanH) / sigmaH;
        double ds = (s - meanS) / sigmaS;
        double dv = (v - meanV) / sigmaV;
        return Math.sqrt(dh * dh + ds * ds + dv * dv);
    }

}
