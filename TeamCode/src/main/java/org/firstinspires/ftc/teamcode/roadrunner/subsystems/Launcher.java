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

public class Launcher {
    private final DcMotorEx launcherMotor;

    private LaunchIntakeSystem launcher = new LaunchIntakeSystem();;


    public Launcher(HardwareMap hwMap, Telemetry tele) {

        launcherMotor = hwMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        launcherMotor.setPower(0);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.init(Constants.launcherPowerSteps, hwMap, tele);
    }

    public void stop() {
        launcherMotor.setPower(0);
    }

    public void farPower(HardwareMap hwMap, Telemetry tele) {
        launcherMotor.setPower(0.73);
        //launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launcher.spinToVelocity(36, tele, hwMap);
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
