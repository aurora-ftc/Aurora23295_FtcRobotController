package org.firstinspires.ftc.teamcode.teleOp.util;

public class MathUtils {
    public static double gaussian3D(double h, double s, double v,
                                    double meanH, double sigmaH,
                                    double meanS, double sigmaS,
                                    double meanV, double sigmaV) {
        double pH = gaussian(h, meanH, sigmaH);
        double pS = gaussian(s, meanS, sigmaS);
        double pV = gaussian(v, meanV, sigmaV);
        return pH * pS * pV;
    }

    public static double gaussian(double x, double mean, double sigma) {
        return Math.exp(-0.5 * Math.pow((x - mean) / sigma, 2));
    }

    public static double mahalanobisDistance(double h, double s, double v,
                                             double meanH, double sigmaH,
                                             double meanS, double sigmaS,
                                             double meanV, double sigmaV) {
        double dh = (h - meanH) / sigmaH;
        double ds = (s - meanS) / sigmaS;
        double dv = (v - meanV) / sigmaV;
        return Math.sqrt(dh * dh + ds * ds + dv * dv);
    }
}
