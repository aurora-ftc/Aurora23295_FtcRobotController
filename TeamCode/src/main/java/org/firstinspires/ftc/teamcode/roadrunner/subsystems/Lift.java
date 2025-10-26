package org.firstinspires.ftc.teamcode.roadrunner.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    // superspeed servo
    public final ServoImplEx lift;

    public Lift(HardwareMap hwMap) {
        lift = (ServoImplEx) hwMap.get(Servo.class, "lift_servo");
        //lift.setPwmRange(new PwmControl.PwmRange(600, 2400));
        lift.scaleRange(0.1, 0.24);
    }

    public class Flick implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private double expectedTime;
        private final double target;

        public Flick(double target, double expectedTime) {
            this.target = target;
            this.expectedTime = expectedTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            lift.setPosition(target);
            return timer.seconds() <= expectedTime;
        }
    }

    public Action lift(double target, double expectedTime) {
        return new Flick(target, expectedTime);
    }
}