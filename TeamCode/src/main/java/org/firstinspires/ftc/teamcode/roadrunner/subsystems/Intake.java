package org.firstinspires.ftc.teamcode.roadrunner.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private final DcMotorEx intakeMotor;

    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() { intakeMotor.setPower(0); }
    public void fullPower() { intakeMotor.setPower(1); }

    // ---------- Actions ----------
    public Action spinForTime(double power, double duration) {
        return new Action() {
            private boolean init = false;
            private final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) timer.reset();
                init = true;
                intakeMotor.setPower(power);
                return timer.seconds() < duration;
            }
        };
    }

    public Action stopAction() {
        return packet -> { intakeMotor.setPower(0); return false; };
    }

    public Action fullPowerAction() {
        return packet -> { intakeMotor.setPower(1); return false; };
    }
}
