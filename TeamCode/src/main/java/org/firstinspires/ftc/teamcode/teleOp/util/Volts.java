package org.firstinspires.ftc.teamcode.teleOp.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Volts {
    // fields / init
    double Vnominal = 12.5;          // voltage where you tuned F
    double vFiltered = Vnominal;
    double alpha = 0.08;             // voltage filter smoothing factor

    //Method to read battery
    public double readBatteryVoltage(HardwareMap hardwareMap) {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0 && v < min) min = v;
        }
        return (min == Double.POSITIVE_INFINITY) ? Vnominal : min;
    }

    public double smoothVolts(double Vread) {
        vFiltered = alpha * Vread + (1.0 - alpha) * vFiltered;
        return vFiltered;
    }


}
