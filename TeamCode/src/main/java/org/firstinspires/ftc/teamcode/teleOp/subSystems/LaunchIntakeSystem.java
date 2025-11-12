package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.ConstantConfig;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;
import org.firstinspires.ftc.teamcode.teleOp.util.Volts;

public class LaunchIntakeSystem {
    private DcMotor intakeMotor;
    private DcMotorEx launcherMotor;
    private VoltageSensor voltageSensor;
    private Servo liftServo;
    private Volts volts = new Volts();
    private final int MIN_STEP = 0;
    private int maxStep = 0;
    private double power, batteryVolts, batteryCorrectedKv;
    private int currentStep = MIN_STEP + 1;
    private double[] powerSteps;
    public boolean launcherOn = false;
    public boolean intakeOn = false;
    private final ElapsedTime time = new ElapsedTime();
    private PIDController flywheelPID;
    private double intakeBlipReset = 0;
    private int intakeSpinDirection = 1;
    private boolean autoPower = false;

    public void init(double servoMin, double servoMax, double[] powerSteps, HardwareMap hwMap, Telemetry telemetry) {
        this.powerSteps = powerSteps;
        maxStep = this.powerSteps.length - 1;

        launcherMotor = hwMap.get(DcMotorEx.class, "launcher_motor");
        intakeMotor = hwMap.get(DcMotor.class, "intake_motor");

        liftServo = hwMap.get(Servo.class, "lift_servo");

        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        liftServo.setDirection(Servo.Direction.FORWARD);

        launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftServo.scaleRange(servoMin, servoMax);

        flywheelPID = new PIDController(ConstantConfig.flywheelKp,
                ConstantConfig.flywheelKi,ConstantConfig.flywheelKd,
                ConstantConfig.flywheelKv, ConstantConfig.flywheelKs);
        flywheelPID.previousTime = System.nanoTime() / 1e9;

        //Set Servo Down
        liftServo.setPosition(1.0);

        debugTelemetry(telemetry);

        time.reset();
    }

    public void spinToVelocity(double targetVelocity, Telemetry tele, HardwareMap hwMap) {

        flywheelPID.setTarget(targetVelocity);

        double currentVelocity = launcherMotor.getVelocity() / 28.0;
        if (ConstantConfig.debug) tele.addData("currentVelocity", currentVelocity);

        double time = System.nanoTime() / 1e9; //Seconds
        batteryCorrectedKv = (ConstantConfig.flywheelKv * 12.5)/ batteryVolts;
        batteryCorrectedKv = Math.min(0.15, Math.max(batteryCorrectedKv, 0.05));

        double outputPID = flywheelPID.calculateOutputPID(currentVelocity, time,
                false);
        double outputFF = flywheelPID.calculateOutputFF(targetVelocity, batteryCorrectedKv);
        double output = outputPID + outputFF;

        launcherMotor.setPower(output);

        if (ConstantConfig.debug) {
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

    private void setLauncherPower(int step, Telemetry tele, double pow, HardwareMap hwMap) {
        if (step >= 0 && step <= maxStep) {
            if (launcherOn) {
                if (!autoPower) spinToVelocity(powerSteps[step], tele, hwMap);
                else spinToVelocity(pow, tele, hwMap);
            } else {
                launcherMotor.setPower(0.0);
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
                batteryVolts : 12.5;
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
        currentStep = Math.max(currentStep - 1, MIN_STEP);
    }

    public void toggleIntake() {
        if (!intakeOn) {
            intakeMotor.setPower(this.intakeSpinDirection);
            intakeOn = true;
        } else {
            intakeMotor.setPower(0);
            intakeOn = false;
        }
    }

    public void toggleIntakeReverse() {
        if (!intakeOn) {
            intakeMotor.setPower(this.intakeSpinDirection * -1);
            intakeOn = true;
        } else {
            intakeMotor.setPower(0);
            intakeOn = false;
        }
    }

    public void intakeBlipReset() {
        intakeBlipReset = time.milliseconds();
    }

    public void intakeBlipLoop() {
        if (time.milliseconds() > 1000 && time.milliseconds() <= intakeBlipReset + 700
                && intakeBlipReset + 200 >= time.milliseconds()) {
            intakeMotor.setPower(1);
        } else {
            if (!intakeOn) intakeMotor.setPower(0);
        }
    }

    public void liftUp() {
        liftServo.setPosition(0.0);
    }

    //No, this is not wrong. The lower position is 1.0 and the upper position is 0.0
    public void liftDown() {
        liftServo.setPosition(1.0);
    }

    public void compTelemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addData("Launcher Status", launcherOn? "On" : "Off");
        telemetry.addData("Intake Status", intakeOn? "On" : "Off");
        telemetry.addLine();
        telemetry.addData("Auto Power", autoPower? "On": "Off");
        telemetry.addData("Launcher Power (%)", !autoPower
                ? (int) (((powerSteps[currentStep] / 85.0) * 100) + 0.5)
                : (int) (((power / 85.0) * 100) + 0.5)
                + "%");
        telemetry.addData("Power Step", !autoPower
                ? powerSteps[currentStep]
                : power);
        telemetry.addLine();
    }

    public void debugTelemetry(Telemetry telemetry) {
        telemetry.addData("Power Step", currentStep);
        telemetry.addData("Intake", intakeMotor.getPower());
        telemetry.addData("Lift Servo Position", liftServo.getPosition());
        telemetry.addLine();
        telemetry.addData("Outtake", launcherOn);
        telemetry.addData("Auto Power", autoPower? "On": "Off");
        telemetry.addData("Launcher Power", autoPower? power: powerSteps[currentStep]);
        telemetry.addLine();
        if (powerSteps != null && currentStep >= 0 && currentStep <= maxStep) {
            telemetry.addData("Launcher Power (%):", autoPower ?
                    (int) (((powerSteps[currentStep] / 85.0) * 100) + 0.5)
                    : (int) ((power / 85.0) + 0.5)
                    + "%");
        }
        telemetry.addLine();
        telemetry.addData("Battery Volts", batteryVolts);
        telemetry.addData("Battery Corrected Kv", batteryCorrectedKv);
        telemetry.addLine();
    }

    public void enableAutoPower() {
        autoPower = true;
    }

    public void disableAutoPower() {
        autoPower = false;
    }

    public void toggleAutoPower() {
        autoPower = !autoPower;
    }

    private double calcAutoPower(double distance) {
        power = 0.0465116 * distance + 37.81395;
        power = Math.max(41, Math.min(44.5, power));
        return power;
    }

}