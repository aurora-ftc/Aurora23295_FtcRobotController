package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.FLYWHEEL_KD;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.FLYWHEEL_KI;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.FLYWHEEL_KP;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.FLYWHEEL_KS;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.FLYWHEEL_KV;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.HWMap;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.LAUNCHER_ENCODER_PER_REV;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.MAX_FLYWHEEL_KV;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.MIN_FLYWHEEL_KV;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.VOLTS_NOMINAL;

import android.service.controls.actions.ModeAction;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;
import org.firstinspires.ftc.teamcode.teleOp.util.Volts;

public class LaunchIntakeSystem {
    private final int minStep = 0;
    private final Volts volts = new Volts();
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final BallSelector ballselector = new BallSelector();
    public boolean launcherOn = false;
    public boolean intakeOn = false;
    private DcMotor intakeMotor;
    private DcMotorEx launcherMotor;
    private PIDController flywheelPID;
    private int maxStep = 0;
    private int currentStep = minStep + 1;
    private double[] powerSteps;
    private double power, batteryVolts, batteryCorrectedKv;
    private boolean autoPower = false;

    /**
     * init: initializes hardware, PID and a ballSelector
     *
     * @param powerSteps the array of power steps (may be unused soon)
     * @param hwMap      hardware map
     */

    public void init(double[] powerSteps, @NonNull HardwareMap hwMap) {
        this.powerSteps = powerSteps;
        maxStep = this.powerSteps.length - 1;

        launcherMotor = hwMap.get(DcMotorEx.class, HWMap.LAUNCHER_MOTOR);
        intakeMotor = hwMap.get(DcMotor.class, HWMap.INTAKE_MOTOR);


        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);


        launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        flywheelPID = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KV, FLYWHEEL_KS);
        flywheelPID.previousTime = System.nanoTime() / 1e9;
        ballselector.init(hwMap);

        intakeTimer.reset();
    }

    /**
     * spinToVelocity: sets the PID to a target velocity
     *
     * @param targetVelocity the target velocity
     */
    public void spinToVelocity(double targetVelocity) {

        flywheelPID.setTarget(targetVelocity);

        double currentVelocity = launcherMotor.getVelocity() / LAUNCHER_ENCODER_PER_REV;

        double time = System.nanoTime() / 1e9; // Seconds
        batteryCorrectedKv = FLYWHEEL_KV * (12.5 / batteryVolts);
        batteryCorrectedKv = Math.min(MAX_FLYWHEEL_KV, Math.max(batteryCorrectedKv, MIN_FLYWHEEL_KV));

        double outputPID = flywheelPID.calculateOutputPID(currentVelocity, time, false);
        double outputFF = flywheelPID.calculateOutputFF(targetVelocity, batteryCorrectedKv);
        double output = outputPID + outputFF;

        launcherMotor.setPower(output);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("target", flywheelPID.target);
        packet.put("current", flywheelPID.current);
        packet.put("output", output);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /**
     * setLauncherPower: sets the launcher power to a given power or automatically
     * if autoPower is true
     *
     * @param step    the power step
     * @param autoPow automatically generated power value
     */
    private void setLauncherPower(int step, double autoPow) {
        if (step >= 0 && step <= maxStep) {
            if (launcherOn) {
                if (autoPower)
                    spinToVelocity(autoPow);
                else
                    spinToVelocity(powerSteps[step]);
            } else {
                // launcherMotor.setPower(0.0);
                spinToVelocity(0);
            }
        }
    }

    /**
     * toggleLauncher: toggles launcher state
     */
    public void toggleLauncher() {
        launcherOn = !launcherOn;
    }

    /**
     * toggleIntake: toggles the intake
     */
    public void toggleIntake() {
        if (!intakeOn) {
            intakeMotor.setPower(1);
            intakeOn = true;
        } else {
            intakeMotor.setPower(0);
            intakeOn = false;
        }
    }

    /**
     * updateLauncher: periodic update for the launch mechanism
     */
    public void updateLauncher(double dist, HardwareMap hwMap) {
        double pow = calcAutoPower(dist);

        batteryVolts = volts.smoothVolts(volts.readBatteryVoltage(hwMap));
        batteryVolts = batteryVolts <= 15 && batteryVolts >= 9 ? batteryVolts : VOLTS_NOMINAL;

        if (launcherOn) {
            setLauncherPower(currentStep, pow);
        } else {
            launcherMotor.setPower(0.0);
        }
    }

    /**
     * stepUpPower and stepDownPower: for use with d-pad controls
     *
     * @author james Beers
     */
    public void stepUpPower() {
        currentStep = Math.min(currentStep + 1, maxStep);
    }

    public void stepDownPower() {
        currentStep = Math.max(currentStep - 1, minStep);
    }


    public void toggleIntakeReverse() {
        if (!intakeOn) {
            intakeMotor.setPower(-1);
            intakeOn = true;
        } else {
            intakeMotor.setPower(0);
            intakeOn = false;
        }
    }

    public void liftUp() {
    }

    // No, this is not wrong. The lower position is 1.0 and the upper position is 0.0
    public void liftDown() {
    }

    public void updateTelemetry(Telemetry telemetry) {
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        multiTelemetry.addLine(">>----> Launch Intake System Telemetry <----<<");
        multiTelemetry.addData("Launcher Status", launcherOn ? "On" : "Off");
        multiTelemetry.addData("Intake Status", intakeOn ? "On" : "Off");
        multiTelemetry.addData("Auto Power", autoPower ? "On" : "Off");
        multiTelemetry.addData("Launcher Power (%)", !autoPower ? (int) (((powerSteps[currentStep] / 85.0) * 100) + 0.5)
                : (int) (((power / 85.0) * 100) + 0.5) + "%");
        multiTelemetry.addData("Power Step", !autoPower ? powerSteps[currentStep] : power);
        multiTelemetry.addData("Battery Volts", batteryVolts);

        if (DEBUG) {
            multiTelemetry.addLine("--- Debug Info ---");
            multiTelemetry.addData("Current Step", currentStep);
            multiTelemetry.addData("Intake Power", intakeMotor.getPower());
            multiTelemetry.addData("Battery Corrected Kv", batteryCorrectedKv);
        }

        multiTelemetry.update();
    }

    public void disableAutoPower() {
        autoPower = false;
    }

    public void toggleAutoPower() {
        autoPower = !autoPower;
    }

    private double calcAutoPower(double distance) {
        power = 0.09 / 65116 * distance + 53.81395;
        power = Math.max(41, Math.min(44.5, power));
        return power;
    }
}