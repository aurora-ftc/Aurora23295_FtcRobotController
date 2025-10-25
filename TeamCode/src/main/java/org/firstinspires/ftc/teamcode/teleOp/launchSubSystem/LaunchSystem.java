package org.firstinspires.ftc.teamcode.teleOp.launchSubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchSystem {
    private DcMotor launcherMotor, intakeMotor;
    private Servo liftServo;
    private final int MIN_STEP = 0;
    private int maxStep = 0;
    int currentStep = MIN_STEP + 1;
    private double[] powerSteps;
    boolean launcherOn = false;
    boolean intakeOn = false;
    private final ElapsedTime time = new ElapsedTime();
    private double intakeBlipReset = 0;
    private int intakeSpinDirection = 1;

    public void init(double servoMin, double servoMax, double[] powerSteps, HardwareMap hwMap, Telemetry telemetry) {
        this.powerSteps = powerSteps;
        maxStep = this.powerSteps.length - 1;

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
        if (step >= 0 && step <= maxStep) {
            if (launcherOn) {
                launcherMotor.setPower(powerSteps[step]);
            } else {
                launcherMotor.setPower(0.0);
            }
        }
    }

    public void toggleLauncher() {
        if (!launcherOn) {
            setLauncherPower(currentStep);
            launcherOn = true;
        } else {
            launcherMotor.setPower(0.0);
            launcherOn = false;
        }
    }

    public void updateLauncher() {
        if (launcherOn) {
            launcherMotor.setPower(powerSteps[currentStep]);
        } else {
            launcherMotor.setPower(0.0);
        }
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


    public void changeIntakeSpinDirection() {
        if (intakeSpinDirection == 1)  {
            intakeSpinDirection = -1;
        } else {
            intakeSpinDirection = 1;
        }
    }

    public void intakeBlipReset() {
        intakeBlipReset = time.milliseconds();
    }

    public void intakeBlipLoop() {
        if (time.milliseconds() > 1000 && time.milliseconds() <= intakeBlipReset + 600
                && intakeBlipReset + 150 >= time.milliseconds()) {
            intakeMotor.setPower(1);
        } else {
            if (!intakeOn) {
                intakeMotor.setPower(0);
            }
        }
    }


    public void liftUp() {
        liftServo.setPosition(0.0);
    }

    //No, this is not wrong. The lower position is 1.0 and the upper position is 0.0
    public void liftDown() {
        liftServo.setPosition(1.0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Power Step", currentStep);
        telemetry.addData("Intake", intakeMotor.getPower());
        telemetry.addData("Lift Servo Position", liftServo.getPosition());
        telemetry.addLine();
        telemetry.addData("Outtake", launcherOn);
        telemetry.addLine();
        if (powerSteps != null && currentStep >= 0 && currentStep <= maxStep) {
            telemetry.addData("Launcher Power:", powerSteps[currentStep] * 100 + "%");
        }
        telemetry.addLine();
    }
}