package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;
import org.firstinspires.ftc.teamcode.teleOp.util.Volts;

public class LaunchIntakeSystem {
    private DcMotor intakeMotor;
    private DcMotorEx launcherMotor;
    private Servo liftServo, angleServo;

    private Volts volts = new Volts();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime liftTimer = new ElapsedTime();
    private PIDController flywheelPID;
    private Indicator indicator = new Indicator();

    private final int minStep = 0;
    private int maxStep = 0;
    private int currentStep = minStep + 1;
    private double[] powerSteps;

    private double power, batteryVolts, batteryCorrectedKv, angle;

    public boolean launcherOn = false;
    public boolean intakeOn = false;
    private boolean autoPowerOn = false;
    private boolean liftOpen = false;

    public void init(double[] powerSteps, HardwareMap hwMap, Telemetry telemetry) {
        this.powerSteps = powerSteps;
        maxStep = this.powerSteps.length - 1;

        launcherMotor = hwMap.get(DcMotorEx.class, HWMap.LAUNCHER_MOTOR);
        intakeMotor = hwMap.get(DcMotor.class, HWMap.INTAKE_MOTOR);

        liftServo = hwMap.get(Servo.class, HWMap.LIFT_SERVO);
        angleServo = hwMap.get(Servo.class, "angle_servo");

        indicator.init(hwMap);

        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        liftServo.setDirection(Servo.Direction.FORWARD);
        angleServo.setDirection(Servo.Direction.REVERSE);

        launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        liftServo.scaleRange(LIFT_SERVO_MIN, LIFT_SERVO_MAX);
        angleServo.scaleRange(0, 1); //MIN down Max UP

        flywheelPID = new PIDController(FLYWHEEL_KP,
                FLYWHEEL_KI, FLYWHEEL_KD,
                FLYWHEEL_KV, FLYWHEEL_KS);
        flywheelPID.previousTime = System.nanoTime() / 1e9;

        debugTelemetry(telemetry);

        intakeTimer.reset();
        liftTimer.reset();

        angleServo.setPosition(0);

        indicator.green();

        liftClose();
    }

    public void spinToVelocity(double targetVelocity, Telemetry tele) {

        flywheelPID.setTarget(targetVelocity);

        double currentVelocity = launcherMotor.getVelocity() / LAUNCHER_ENCODER_PER_REV;
        currentVelocity *= -1;
        if (DEBUG) tele.addData("currentVelocity", currentVelocity);

        double time = System.nanoTime() / 1e9; //Seconds
        batteryCorrectedKv = FLYWHEEL_KV * (12.5 / batteryVolts);
        batteryCorrectedKv = Math.min(MAX_FLYWHEEL_KV, Math.max(batteryCorrectedKv, MIN_FLYWHEEL_KV));

        double outputPID = flywheelPID.calculateOutputPID(currentVelocity, time, false);
        double outputFF = flywheelPID.calculateOutputFF(targetVelocity, batteryCorrectedKv);
        double output = outputPID + outputFF;

        launcherMotor.setPower(-output);

        flywheelPID.setKV(FLYWHEEL_KV);
        flywheelPID.setKP(FLYWHEEL_KP);
        flywheelPID.setKI(FLYWHEEL_KI);
        flywheelPID.setKD(FLYWHEEL_KD);

        if (DEBUG) {
            tele.addData("output total", output);
            tele.addData("outputFF", outputFF);
            tele.addData("outputPID", outputPID);
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("target",flywheelPID.target);
        packet.put("current", flywheelPID.current);
        packet.put("output", output);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void setLauncherPower(int step, Telemetry tele, double autoPow, HardwareMap hwMap) {
        if (step >= 0 && step <= maxStep) {
            if (launcherOn) {
                if (autoPowerOn) spinToVelocity(autoPow, tele);
//                else spinToVelocity(powerSteps[step], tele);
                else spinToVelocity(46, tele);
            } else {
                launcherMotor.setPower(0.0);
                //spinToVelocity(0, tele);
            }
        }
    }

    public void toggleLauncher() {
        if (!launcherOn) {
            launcherOn = true;
        } else {
            launcherOn = false;
        }
    }

    public void updateLauncher(Telemetry tele, double dist, HardwareMap hwMap) {

        if (Math.abs(flywheelPID.current - flywheelPID.target) <= 1.5 && intakeMotor.getPower() >= 0.5 && launcherMotor.getPower() != 0)
            indicator.green();
        else if (Math.abs(flywheelPID.current - flywheelPID.target) <= 1.5 && launcherMotor.getPower() != 0)
            indicator.yellow();
        else
            indicator.red();

        double pow = calcAutoPower(dist);
        double ang = calcAutoHood(dist);

        batteryVolts = volts.smoothVolts(volts.readBatteryVoltage(hwMap));
        batteryVolts = batteryVolts <= 15 && batteryVolts >= 9?
                batteryVolts : VOLTS_NOMINAL;

        tele.addData("Battery Volts", batteryVolts);

        setLauncherPower(currentStep, tele, pow, hwMap);

        if (!autoPowerOn)
            ang = 0;

        angleServo.setPosition(ang);

        if (liftTimer.milliseconds() > 1000 && liftOpen)
            liftClose();
    }

    public void stepUpPower() {
        currentStep = Math.min(currentStep + 1, maxStep);
    }

    public void stepDownPower() {
        currentStep = Math.max(currentStep - 1, minStep);
    }

    public void toggleIntake() {
        if (!intakeOn) {
            intakeMotor.setPower(1);
            intakeOn = true;
        } else {
            intakeMotor.setPower(0);
            intakeOn = false;
        }
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

    public void intakeBlipLoop() {
        if (200 < intakeTimer.milliseconds() &&
                intakeTimer.milliseconds() < 600) {
            intakeMotor.setPower(1);
        } else {
            if (!intakeOn) intakeMotor.setPower(0);
        }
    }

    public void liftBlip() {
        liftTimer.reset();
        liftOpen();
    }

    public void liftOpen() {
        liftServo.setPosition(LIFT_SERVO_OPEN);
        liftOpen = true;
    }

    //No, this is not wrong. The lower position is 1.0 and the upper position is 0.0
    public void liftClose() {
        liftServo.setPosition(LIFT_SERVO_CLOSE);
        liftOpen = false;
    }

    public void compTelemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addData("Auto Features", autoPowerOn ? "On": "Off");
        telemetry.addData("Power Step", !autoPowerOn
                ? powerSteps[currentStep]
                : power);
        telemetry.addLine();
        telemetry.addData("angle", angle);
        telemetry.addData("power", power);
    }

    public void debugTelemetry(Telemetry telemetry) {
        telemetry.addData("Current Step", currentStep);
        telemetry.addData("Intake", intakeMotor.getPower());
        telemetry.addData("Lift Servo Position", liftServo.getPosition());
        telemetry.addLine();
        telemetry.addData("Outtake", launcherOn);
        telemetry.addData("Auto Power", autoPowerOn ? "On": "Off");
        telemetry.addLine();
        telemetry.addData("Launcher Power (%)", !autoPowerOn
                ? (int) (((powerSteps[currentStep] / 85.0) * 100) + 0.5)
                : (int) (((power / 85.0) * 100) + 0.5)
                + "%");
        telemetry.addData("Power Step", !autoPowerOn
                ? powerSteps[currentStep]
                : power);
        telemetry.addLine();
        telemetry.addData("Battery Volts", batteryVolts);
        telemetry.addData("Battery Corrected Kv", batteryCorrectedKv);
        telemetry.addLine();
    }

    public void disableAutoPower() {
        autoPowerOn = false;
    }

    public void toggleAutoPower() {
        autoPowerOn = !autoPowerOn;
    }

    private double calcAutoPower(double x) {
        power = (((-2.2941e-7 * x
                + 0.00010807) * x
                - 0.0182732) * x
                + 1.46786) * x
                + 2.86572;
        return power;
    }

    private double calcAutoHood(double x) {
        angle = (((-1.04061e-6 * x
                + 0.000438459) * x
                - 0.0687862) * x
                + 4.78304) * x
                - 124.39506;
        angle = Math.min(1, Math.max(0, angle));
        if (x > 124) angle = 1;
        return angle;
    }

}