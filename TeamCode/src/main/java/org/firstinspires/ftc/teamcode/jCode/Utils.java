package org.firstinspires.ftc.teamcode.jCode;

/**<p>Before you change anything...</p>
 * <b>DID YOU READ MY README? </b>
 * <p>thank you for your understanding</p>
 */

public class Utils {
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
    public static double safeSqrt(double n) {
        int sign = 1;

        if (n < 0) {
            sign = -1;
            n *= -1;
        }

        n = Math.sqrt(n);
        n *= sign;

        return n;
    }
}
