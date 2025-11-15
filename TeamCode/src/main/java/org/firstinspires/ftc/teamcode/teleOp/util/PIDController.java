package org.firstinspires.ftc.teamcode.teleOp.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PIDController {
    public double kp;
    public double ki;
    public double kd;
    public double kv;
    public double ks;

    public double target;
    public double current;
    public double output;
    private double integral;
    public double previousError;
    public double previousTime; // Using System.nanoTime() or ElapsedTime for more accurate timing

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        target = 0.0;
        integral = 0.0;
        previousError = 0.0;// Initialize with current time in actual implementation
    }

    public PIDController(double kp, double ki, double kd, double kv, double ks) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kv = kv;
        this.ks = ks;
        target = 0.0;
        integral = 0.0;
        previousError = 0.0;// Initialize with current time in actual implementation
    }

    public void setKP(double kp) {
        this.kp = kp;
    }

    public void setKI(double ki) {
        this.ki = ki;
    }

    public void setKD(double kd) {
        this.kd = kd;
    }

    public void setKV(double kv) {
        this.kv = kv;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double calculateOutputPID(double current, double time, boolean shouldRadianWrap) {
        this.current = current;
        double error = shouldRadianWrap ?
                AngleUnit.normalizeRadians(target - current)
                : (target - current);
        double deltaTime = time - previousTime;

        if (deltaTime <= 0) return output;

        double derivative = (error - previousError) / deltaTime;

        // prevent integral wind-up
        integral += error * deltaTime;
        integral = Math.max(-1, Math.min(1, integral));

        output = kp * error + ki * integral + kd * derivative;

        previousError = error;
        previousTime = time;

        return output;
    }

    public double calculateOutputFF(double targetVelocity, double kv) {
        double FeedForwardOutput = kv * targetVelocity + ks;
        return FeedForwardOutput;
    }

}