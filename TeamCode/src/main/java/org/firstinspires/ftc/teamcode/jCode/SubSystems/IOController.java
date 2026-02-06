package org.firstinspires.ftc.teamcode.jCode.SubSystems;

import static org.firstinspires.ftc.teamcode.jCode.Configuration.FLYWHEEL_KD;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.FLYWHEEL_KF;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.FLYWHEEL_KI;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.FLYWHEEL_KP;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.HWMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IOController {
    private final ElapsedTime timer = new ElapsedTime();
    boolean intakeStatus, outtakeStatus;
    private DcMotor intake;
    private DcMotorEx outtake;
    private Servo pushServo;

    public void init(HardwareMap map) {
        intake = map.get(DcMotor.class, HWMap.intakeMotor);
        outtake = map.get(DcMotorEx.class, HWMap.outtakeMotor);

        pushServo = map.get(Servo.class, HWMap.pushServo);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pushServo.scaleRange(0.05, 0.25);
        pushServo.setPosition(0.0);
    }

    public void toggleIntake() {
        if (!intakeStatus) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        intakeStatus = !intakeStatus;
    }

    public void toggleOuttake() {
        outtakeStatus = !outtakeStatus;
    }

    public void push() {
        pushServo.setPosition(1.0);
        timer.reset();
    }

    public void periodic() {
        if (timer.milliseconds() > 50) {
            pushServo.setPosition(0);
        }
        if (!outtakeStatus) outtake.setVelocity(1, AngleUnit.RADIANS);//FIX THIS
        if (outtakeStatus) outtake.setPower(0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("IO Controller");
        telemetry.addData("IO Status", "OK");
        telemetry.addData("Outtake status", outtakeStatus ? "ON" : "OFF");
        telemetry.addData("Intake status", intakeStatus ? "ON" : "OFF");
    }
}
