package org.firstinspires.ftc.teamcode.roadrunner.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private final DcMotor launcherMotor;

    public Launcher(HardwareMap hwMap) {
        launcherMotor = hwMap.get(DcMotor.class, "launcher_motor");
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcherMotor.setPower(0);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        launcherMotor.setPower(0);
    }

    public void farPower() {
        launcherMotor.setPower(0.8);
    }

    public void setPower(double power) {
        launcherMotor.setPower(power);
    }

    // ---------- Actions ----------
    public Action spinForTime(double power, double duration) {
        return new Action() {
            private boolean init = false;
            private final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    timer.reset();
                    init = true;
                    packet.put("Timer", "Reset");
                }

                launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (timer.seconds() < duration) {
                    launcherMotor.setPower(power);
                    packet.put("Power", power);
                    return true;
                } else {
                    launcherMotor.setPower(0);
                    packet.put("Power", power);
                    return false;
                }
            }
        };
    }

}
