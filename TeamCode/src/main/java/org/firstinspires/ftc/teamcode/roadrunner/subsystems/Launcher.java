package org.firstinspires.ftc.teamcode.roadrunner.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    //6000rpm motor
    private final DcMotorEx launcherMotor;

    public Launcher(HardwareMap hwMap) {
        launcherMotor = hwMap.get(DcMotorEx.class, "launcher_motor");

        launcherMotor.setPower(0);
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public class Launch implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private final double expectedTime;
        private final double powerVal;

        public Launch(double powerVal, double expectedTime) {
            this.powerVal = powerVal;
            this.expectedTime = expectedTime;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setPower(powerVal);

            return timer.seconds() <= expectedTime;
        }

    }

    public Action launch(double powerVal, double expectedTime) {
        return new Launch(powerVal, expectedTime);
    }
}