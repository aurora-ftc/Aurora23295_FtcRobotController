package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "HSV_Tuner", group = "TestModes")
public class HSV_Tuner extends LinearOpMode {

    // Adjust defaults if you want
    private static final String SENSOR_NAME = "color_sensor";
    private static final int DEFAULT_MAX_SAMPLES = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, SENSOR_NAME);
        RevColorSensorV3 rev = (RevColorSensorV3) colorSensor;
        rev.setGain(10);

        waitForStart();

        telemetry.addLine("HSV Tuner ready.");
        telemetry.addLine("Press A to START sampling. Press B to STOP and compute.");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                // start sampling loop until user presses B
                sampleAndCompute(colorSensor);
            }

            if (gamepad1.y) {
                telemetry.addLine("Exiting tuner.");
                telemetry.update();
                break;
            }

            idle();
        }
    }

    private void sampleAndCompute(ColorSensor sensor) {
        telemetry.clearAll();
        telemetry.addLine("SAMPLING... Press B to stop.");
        telemetry.update();

        // accumulators for S and V (linear)
        double sumS = 0.0, sumSsq = 0.0;
        double sumV = 0.0, sumVsq = 0.0;

        // For hue (circular) we accumulate as unit vectors
        double sumCos = 0.0, sumSin = 0.0;

        int samples = 0;
        float[] hsv = new float[3];

        // loop until B pressed or max samples
        while (opModeIsActive() && !gamepad1.b && samples < DEFAULT_MAX_SAMPLES) {
            int r = sensor.red();
            int g = sensor.green();
            int b = sensor.blue();

            // protect against zero-sum
            double sumRGB = r + g + b;
            if (sumRGB == 0) sumRGB = 1;

            double rN = r / sumRGB * 255.0;
            double gN = g / sumRGB * 255.0;
            double bN = b / sumRGB * 255.0;

            android.graphics.Color.RGBToHSV((int) rN, (int) gN, (int) bN, hsv);
            // hsv[0] in degrees 0..360, hsv[1] and hsv[2] in 0..1
            double hueDegrees360 = hsv[0];           // 0..360
            double hueForModel = hueDegrees360 / 2.0; // convert to 0..180 (if you use that)

            double sat = hsv[1] * 255.0; // 0..255
            double val = hsv[2] * 255.0; // 0..255

            // Circular accumulation for hue: convert degrees to radians
            double hueRad = Math.toRadians(hueDegrees360);
            sumCos += Math.cos(hueRad);
            sumSin += Math.sin(hueRad);

            // linear accumulators for S and V
            sumS += sat;
            sumSsq += sat * sat;

            sumV += val;
            sumVsq += val * val;

            samples++;

            if (samples % 50 == 0) {
                telemetry.addData("Samples", samples);
                telemetry.addData("Latest HSV", "%.1f, %.1f, %.1f", hueForModel, sat, val);
                telemetry.update();
            }

            // small delay to avoid spamming CPU (adjust if you want faster sampling)
            sleep(8);
        }

        // compute means & stds
        if (samples == 0) {
            telemetry.addLine("No samples collected (press A then B to sample).");
            telemetry.update();
            return;
        }

        // Hue: circular mean
        double meanHueRad = Math.atan2(sumSin / samples, sumCos / samples); // -pi..pi
        if (meanHueRad < 0) meanHueRad += 2 * Math.PI; // 0..2pi
        double meanHueDeg360 = Math.toDegrees(meanHueRad); // 0..360
        double meanHueForModel = meanHueDeg360 / 2.0; // 0..180 (matches detector)

        // circular standard deviation (approx)
        // R = sqrt((sumCos/n)^2 + (sumSin/n)^2)
        double rbar = Math.sqrt(Math.pow(sumCos / samples, 2) + Math.pow(sumSin / samples, 2));
        // circular variance estimator: var = 1 - rbar
        double circularVar = 1.0 - rbar;
        // approximate circular standard deviation (in radians): sqrt(-2 ln rbar)
        double circStdRad = 0.0;
        if (rbar > 0) {
            circStdRad = Math.sqrt(-2.0 * Math.log(rbar));
        } else {
            // rbar == 0 (uniform) => large std
            circStdRad = Math.PI / Math.sqrt(3.0); // fallback
        }
        double circStdDeg360 = Math.toDegrees(circStdRad); // degrees on 0..360
        double circStdForModel = circStdDeg360 / 2.0; // convert to 0..180 scale

        // S and V means and std (linear)
        double meanS = sumS / samples;
        double meanV = sumV / samples;

        double varS = (sumSsq / samples) - (meanS * meanS);
        double varV = (sumVsq / samples) - (meanV * meanV);
        if (varS < 0) varS = 0;
        if (varV < 0) varV = 0;
        double stdS = Math.sqrt(varS);
        double stdV = Math.sqrt(varV);

        // enforce minimal sigma so your detector isn't too tight
        double minSigmaH = 6.0;   // degrees in 0..180 scale (tweak)
        double minSigmaSV = 12.0; // for S and V (tweak)

        double suggestedSigmaH = Math.max(circStdForModel, minSigmaH);
        double suggestedSigmaS = Math.max(stdS, minSigmaSV);
        double suggestedSigmaV = Math.max(stdV, minSigmaSV);

        // Print results
        telemetry.clearAll();
        telemetry.addData("Samples Collected", samples);
        telemetry.addLine("");

        telemetry.addData("Mean Hue (0..180)", "%.2f", meanHueForModel);
        telemetry.addData("CircStd Hue (0..180)", "%.2f", circStdForModel);
        telemetry.addData("Suggested SigmaH", "%.2f", suggestedSigmaH);
        telemetry.addLine("");

        telemetry.addData("Mean Sat (0..255)", "%.2f", meanS);
        telemetry.addData("Std Sat", "%.2f", stdS);
        telemetry.addData("Suggested SigmaS", "%.2f", suggestedSigmaS);
        telemetry.addLine("");

        telemetry.addData("Mean Val (0..255)", "%.2f", meanV);
        telemetry.addData("Std Val", "%.2f", stdV);
        telemetry.addData("Suggested SigmaV", "%.2f", suggestedSigmaV);
        telemetry.addLine("");

        telemetry.addLine("Copy these means & sigmas into your detector.");
        telemetry.update();

        // beep or wait so user can read
        sleep(1000);
    }
}
