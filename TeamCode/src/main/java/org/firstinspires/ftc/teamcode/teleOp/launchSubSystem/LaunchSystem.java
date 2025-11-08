package org.firstinspires.ftc.teamcode.teleOp.launchSubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.ConstantConfig;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.PIDController;

public class LaunchSystem {
    private DcMotor intakeMotor;
    private DcMotorEx launcherMotor;
    private Servo liftServo;
    private final int MIN_STEP = 0;
    private int maxStep = 0;
    int currentStep = MIN_STEP + 1;
    private double[] powerSteps;
    public boolean launcherOn = false;
    public boolean intakeOn = false;
    private final ElapsedTime time = new ElapsedTime();
    private PIDController flywheelPID;
    private double intakeBlipReset = 0;
    private int intakeSpinDirection = 1;

    public void init(double servoMin, double servoMax, double[] powerSteps, HardwareMap hwMap, Telemetry telemetry) {
        this.powerSteps = powerSteps;
        maxStep = this.powerSteps.length - 1;

        launcherMotor = hwMap.get(DcMotorEx.class, "launcher_motor");
        intakeMotor = hwMap.get(DcMotor.class, "intake_motor");

        liftServo = hwMap.get(Servo.class, "lift_servo");

        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        liftServo.setDirection(Servo.Direction.FORWARD);

        //launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //PIDFCoefficients pidf = new PIDFCoefficients(0,0,0,1);
        //launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftServo.scaleRange(servoMin, servoMax);

        flywheelPID = new PIDController(0.1,0,0,0.15);
        flywheelPID.setTarget(0.65);
        flywheelPID.previousTime = System.nanoTime() / 1e9;

        //Set Servo Down
        liftServo.setPosition(1.0);

        debugTelemetry(telemetry);

        time.reset();
    }

    public void spinToVelocity(double targetVelocity, double kp, double ki, double kd, double kv, Telemetry tele) {

        flywheelPID.setKP(kp);
        flywheelPID.setKI(ki);
        flywheelPID.setKD(kd);
        flywheelPID.setKV(kv);
        flywheelPID.setTarget(targetVelocity);

        double currentVelocity = launcherMotor.getVelocity(AngleUnit.RADIANS);
        tele.addData("currentVelocity", currentVelocity);
        double time = System.nanoTime() / 1e9; //Seconds

        double outputPID = flywheelPID.calculateOutput(currentVelocity, time);
        double outputFF = flywheelPID.calculateOutputFF(targetVelocity);
        double output = outputPID + outputFF;

        tele.addData("output total", output);
        tele.addData("outputFF", outputFF);
        tele.addData("outputPID", outputPID);

        launcherMotor.setPower(output);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("target", flywheelPID.target);
        packet.put("current", flywheelPID.current);
        packet.put("output", flywheelPID.output);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }

    public void powerWithPid(double input) {
        launcherMotor.setVelocity(input, AngleUnit.RADIANS);
    }

    private void setLauncherPower(int step) {
        if (step >= 0 && step <= maxStep) {
            if (launcherOn) {
                /*
                TelemetryPacket packet = new TelemetryPacket();

                packet.put("target", powerSteps[step] * 50);
                packet.put("current", launcherMotor.getVelocity(AngleUnit.RADIANS));
                packet.put("output", launcherMotor.getPower());

                //Crucial line: Sends data to FTC Dash
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                //powerWithPid(powerSteps[step] * 50);*/
                launcherMotor.setPower(powerSteps[step]);
            } else {
                launcherMotor.setPower(0.0);
            }
        }
    }

    public void toggleLauncher() {
        if (!launcherOn) {
            //setLauncherPower(currentStep);
            //powerSteps[currentStep]
            //spinToVelocity(0.65, ConstantConfig.flywheelKp,
                   // ConstantConfig.flywheelKi, ConstantConfig.flywheelKd, ConstantConfig.flywheelKv);
            launcherOn = true;
        } else {
            //launcherMotor.setPower(0.0);
            launcherOn = false;
        }
    }

    public void updateLauncher() {
        if (launcherOn) {
            setLauncherPower(currentStep);
            //spinToVelocity(0.65, ConstantConfig.flywheelKp,
                   // ConstantConfig.flywheelKi, ConstantConfig.flywheelKd,
                   // ConstantConfig.flywheelKv);
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

    public void compTelemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addData("Power Step", currentStep);
        telemetry.addLine();
        if (powerSteps != null && currentStep >= 0 && currentStep <= maxStep) {
            telemetry.addData("Launcher Power:", powerSteps[currentStep] * 100 + "%");
        }
        telemetry.addLine();
    }

    public void debugTelemetry(Telemetry telemetry) {
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