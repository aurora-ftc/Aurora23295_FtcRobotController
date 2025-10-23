package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchSystem {
    private DcMotor launcherMotor, intakeMotor;
    private Servo liftServo;
    private final int MIN_STEP = 1;
    private int maxStep = 0;
    int currentStep = MIN_STEP;
    private double[] powerSteps;
    boolean launcherOn = false;
    boolean intakeOn = false;
    private final ElapsedTime time = new ElapsedTime();

    public void init(double servoMin, double servoMax, double[] steps, HardwareMap hwMap, Telemetry telemetry) {
        powerSteps = steps;
        maxStep = powerSteps.length;

        launcherMotor = hwMap.get(DcMotor.class, "launcher_motor");
        intakeMotor = hwMap.get(DcMotor.class, "intake_motor");

        liftServo = hwMap.get(Servo.class, "lift_servo");

        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        liftServo.setDirection(Servo.Direction.FORWARD);

        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftServo.scaleRange(servoMin, servoMax);

        //Set Servo Down
        liftServo.setPosition(1.0);

        updateTelemetry(telemetry);

        time.reset();
    }

    private void setLauncherPower(int step) {
        if (step >= 0 && step < powerSteps.length) {
            if (launcherOn) {
                launcherMotor.setPower(powerSteps[step]);
            } else {
                launcherMotor.setPower(0.0);
            }
        }
    }

    public void toggleLauncher() {
        if (launcherOn) {
            setLauncherPower(currentStep);
        } else {
            launcherMotor.setPower(0.0);
        }
        launcherOn = !launcherOn;
    }

    public void stepUpPower() {
        currentStep = Math.min(currentStep + 1, maxStep);
        setLauncherPower(currentStep);
    }

    public void stepDownPower() {
        currentStep = Math.max(currentStep - 1, MIN_STEP);
        setLauncherPower(currentStep);
    }

    public void toggleIntake() {
        if (!intakeOn) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
        intakeOn = !intakeOn;
    }

    public void liftUp() {
        liftServo.setPosition(0.0);
    }

    //No, this is not wrong. The lower position is 1.0 and the upper position is 0.0
    public void liftDown() {
        liftServo.setPosition(1.0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Launcher", launcherMotor.getPower());
        telemetry.addData("Power Step", currentStep);
        telemetry.addData("Intake", intakeMotor.getPower());
        telemetry.addData("Lift Servo Position", liftServo.getPosition());
        telemetry.addLine();
        telemetry.addData("Outtake", launcherOn);
        telemetry.addLine();
    }
}