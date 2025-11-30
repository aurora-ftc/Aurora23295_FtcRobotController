package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;
import org.firstinspires.ftc.teamcode.teleOp.util.Volts;

public class LaunchIntakeSystem {
    private DcMotor intakeMotor;
    private DcMotorEx launcherMotor;
    private Servo liftServo;

    private Volts volts = new Volts();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private PIDController flywheelPID;

    private final int minStep = 0;
    private int maxStep = 0;
    private int currentStep = minStep + 1;
    private double[] powerSteps;

    private double power, batteryVolts, batteryCorrectedKv;

    public boolean launcherOn = false;
    public boolean intakeOn = false;
    private final ElapsedTime time = new ElapsedTime();

    private double intakeBlipReset = 0;
    private boolean autoPowerOn = false;

    public void init(double[] powerSteps, HardwareMap hwMap, Telemetry telemetry) {
        this.powerSteps = powerSteps;
        maxStep = this.powerSteps.length - 1;

        launcherMotor = hwMap.get(DcMotorEx.class, HWMap.LAUNCHER_MOTOR);
        intakeMotor = hwMap.get(DcMotor.class, HWMap.INTAKE_MOTOR);

        liftServo = hwMap.get(Servo.class, HWMap.LIFT_SERVO);

        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        liftServo.setDirection(Servo.Direction.FORWARD);

        launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        liftServo.scaleRange(LIFT_SERVO_MIN, LIFT_SERVO_MAX);

        flywheelPID = new PIDController(FLYWHEEL_KP,
                FLYWHEEL_KI, FLYWHEEL_KD,
                FLYWHEEL_KV, FLYWHEEL_KS);
        flywheelPID.previousTime = System.nanoTime() / 1e9;

        //Set Servo Down
        liftServo.setPosition(LIFT_SERVO_DOWN);

        debugTelemetry(telemetry);

        intakeTimer.reset();
    }

    public void spinToVelocity(double targetVelocity, Telemetry tele) {

        flywheelPID.setTarget(targetVelocity);

        double currentVelocity = launcherMotor.getVelocity() / LAUNCHER_ENCODER_PER_REV;
        if (DEBUG) tele.addData("currentVelocity", currentVelocity);

        double time = System.nanoTime() / 1e9; //Seconds
        batteryCorrectedKv = FLYWHEEL_KV * (12.5 / batteryVolts);
        batteryCorrectedKv = Math.min(MAX_FLYWHEEL_KV, Math.max(batteryCorrectedKv, MIN_FLYWHEEL_KV));

        double outputPID = flywheelPID.calculateOutputPID(currentVelocity, time, false);
        double outputFF = flywheelPID.calculateOutputFF(targetVelocity, batteryCorrectedKv);
        double output = outputPID + outputFF;

        launcherMotor.setPower(output);

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
                else spinToVelocity(powerSteps[step], tele);
            } else {
                //launcherMotor.setPower(0.0);
                spinToVelocity(0, tele);
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
        double pow = calcAutoPower(dist);

        batteryVolts = volts.smoothVolts(volts.readBatteryVoltage(hwMap));
        batteryVolts = batteryVolts <= 15 && batteryVolts >= 9?
                batteryVolts : VOLTS_NOMINAL;

        tele.addData("Battery Volts", batteryVolts);

        if (launcherOn) {
            setLauncherPower(currentStep, tele, pow, hwMap);
        } else {
            launcherMotor.setPower(0.0);
        }
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

    public void intakeBlipReset() {
        intakeTimer.reset();
    }

    public void intakeBlipLoop() {
        if (200 < intakeTimer.milliseconds() &&
                intakeTimer.milliseconds() < 600) {
            intakeMotor.setPower(1);
        } else {
            if (!intakeOn) intakeMotor.setPower(0);
        }
    }

    public void liftUp() {
        liftServo.setPosition(LIFT_SERVO_UP);
    }

    //No, this is not wrong. The lower position is 1.0 and the upper position is 0.0
    public void liftDown() {
        liftServo.setPosition(LIFT_SERVO_DOWN);
    }

    public void compTelemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addData("Launcher Status", launcherOn? "On" : "Off");
        telemetry.addData("Intake Status", intakeOn? "On" : "Off");
        telemetry.addLine();
        telemetry.addData("Auto Power", autoPowerOn ? "On": "Off");
        telemetry.addData("Launcher Power (%)", !autoPowerOn
                ? (int) (((powerSteps[currentStep] / 85.0) * 100) + 0.5)
                : (int) (((power / 85.0) * 100) + 0.5)
                + "%");
        telemetry.addData("Power Step", !autoPowerOn
                ? powerSteps[currentStep]
                : power);
        telemetry.addLine();
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

    private double calcAutoPower(double distance) {
        power = 0.189189 * distance +38.97297;
        power = Math.max(55, Math.min(65, power));
//        power = 0.09/65116 * distance + 53.81395;
//        power = Math.max(41, Math.min(44.5, power));
        return power;
    }

}