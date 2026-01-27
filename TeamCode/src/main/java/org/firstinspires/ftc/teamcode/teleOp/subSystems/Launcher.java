package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;
import org.firstinspires.ftc.teamcode.teleOp.util.Volts;

public class Launcher {
    //Hardware
    private DcMotorEx launcher;

    //Classes
    private final Volts volts = new Volts();
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private PIDController flywheelPID;

    //Launcher
    private double batteryVolts, batteryCorrectedKv;
    private double[] powerSteps = POWER_STEPS;
    private double power = 0;
    private int maxStep = powerSteps.length - 1, currentStep = 1, minStep = 0;
    boolean launcherOn = false, autoPower = false;

    public void init(HardwareMap hwMap) {
        launcher = hwMap.get(DcMotorEx.class, HWMap.LAUNCHER_MOTOR);

        launcher.setPower(0);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheelPID = new PIDController(PID.Flywheel.KP, PID.Flywheel.KI, PID.Flywheel.KD, PID.Flywheel.KV, PID.Flywheel.KS);
        flywheelPID.previousTime = System.nanoTime() / 1e9;
    }

    public void spinToVelocity(double targetVelocity) {

        flywheelPID.setTarget(targetVelocity);

        double currentVelocity = launcher.getVelocity() / LAUNCHER_ENCODER_PER_REV;

        double time = System.nanoTime() / 1e9; // Seconds

        //Adjusts the necessary kV of the wheel based on
        batteryCorrectedKv = PID.Flywheel.KV * (VOLTS_NOMINAL / batteryVolts);
        batteryCorrectedKv = Math.min(PID.Flywheel.MAX_KV, Math.max(batteryCorrectedKv, PID.Flywheel.MIN_KV));

        double outputPID = flywheelPID.calculateOutputPID(currentVelocity, time, false);
        double outputFF = flywheelPID.calculateOutputFF(targetVelocity, batteryCorrectedKv);
        double output = outputPID + outputFF;

        launcher.setPower(output);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("target", flywheelPID.target);
        packet.put("current", flywheelPID.current);
        packet.put("output", output);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void setLauncherPower(int step, double autoPow) {
        if (step >= 0 && step <= maxStep) {
            if (launcherOn) {
                if (autoPower)
                    spinToVelocity(autoPow);
                else
                    spinToVelocity(powerSteps[step]);
            } else {
                spinToVelocity(0);
            }
        }
    }

    public void toggleLauncher() {
        launcherOn = !launcherOn;
    }

    public void updateLauncher(double dist, HardwareMap hwMap) {
        double pow = calcAutoPower(dist);

        batteryVolts = volts.smoothVolts(volts.readBatteryVoltage(hwMap));
        batteryVolts = batteryVolts <= 15 && batteryVolts >= 9 ? batteryVolts : VOLTS_NOMINAL;

        if (launcherOn)
            setLauncherPower(currentStep, pow);
        else
            launcher.setPower(0.0);
    }

    public void log(Telemetry tele) {
        tele.addLine();

        tele.addLine(">>----> Launch Intake System Telemetry <----<<");
        tele.addData("Launcher Status", launcherOn ? "On" : "Off");
        tele.addData("Auto Power", autoPower ? "On" : "Off");
        tele.addData("Launcher Power (%)", !autoPower ? (int) (((powerSteps[currentStep] / 85.0) * 100) + 0.5)
                : (int) (((power / 85.0) * 100) + 0.5) + "%");
        tele.addData("Power Step", !autoPower ? powerSteps[currentStep] : power);
        tele.addData("Battery Volts", batteryVolts);

        if (DEBUG) {
            tele.addLine("--- Debug Info ---");
            tele.addData("Current Step", currentStep);
            tele.addData("Battery Corrected Kv", batteryCorrectedKv);
        }

        tele.addLine();
    }

    public void stepUpPower() {
        currentStep = Math.min(currentStep + 1, maxStep);
    }

    public void stepDownPower() {
        currentStep = Math.max(currentStep - 1, minStep);
    }
    public void disableAutoPower() {
        autoPower = false;
    }

    public void toggleAutoPower() {
        autoPower = !autoPower;
    }

    private double calcAutoPower(double distance) {
        double power = 0.09 / 65116 * distance + 53.81395;
        power = Math.max(41, Math.min(44.5, power));
        return power;
    }

}
