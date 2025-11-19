package org.firstinspires.ftc.teamcode.teleOp.util;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Volts {
    double vFiltered = VOLTS_NOMINAL;
    double alpha = 0.08;             // voltage filter smoothing factor

    //Method to read battery
    public double readBatteryVoltage(HardwareMap hwMap) {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : hwMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0 && v < min) min = v;
        }
        return (min == Double.POSITIVE_INFINITY) ? VOLTS_NOMINAL : min;
    }

    public double smoothVolts(double vRead) {
        vFiltered = alpha * vRead + (1.0 - alpha) * vFiltered;
        return vFiltered;
    }
}
