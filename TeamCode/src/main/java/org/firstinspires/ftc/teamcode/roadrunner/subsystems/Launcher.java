package org.firstinspires.ftc.teamcode.roadrunner.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;
import org.firstinspires.ftc.teamcode.teleOp.util.Volts;

public class Launcher {
    private final DcMotor launcherMotor;
    private final LaunchIntakeSystem launcher = new LaunchIntakeSystem();
    private final Volts volts = new Volts();


    public Launcher(HardwareMap hwMap) {

        launcherMotor = hwMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        launcher.init(Constants.POWER_STEPS, hwMap);

        launcherMotor.setPower(0);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.init(Constants.POWER_STEPS, hwMap);
    }

    public void stop() {
        launcherMotor.setPower(0);
    }

    public void farPower(HardwareMap hwMap) {
        double batteryVolts = volts.smoothVolts(volts.readBatteryVoltage(hwMap));
        batteryVolts = batteryVolts < 9.5 || batteryVolts > 15 ? 12 : batteryVolts;
        double power = -0.015 * batteryVolts + 0.895;
        power = Math.min(0.75, Math.max(0.68, power));
        launcherMotor.setPower(power);
    }

    public void setPower(double power) {
        launcherMotor.setPower(power);
    }

    // ---------- Actions ----------
    public Action spinForTime(double power, double duration, Telemetry tele) {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    timer.reset();
                    init = true;
                    packet.put("Timer", "Reset");
                }

                launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (timer.seconds() < duration) {
                    launcher.spinToVelocity(power, tele);
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
