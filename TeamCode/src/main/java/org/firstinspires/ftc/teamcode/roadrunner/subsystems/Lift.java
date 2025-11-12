package org.firstinspires.ftc.teamcode.roadrunner.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Lift {
    private final ServoImplEx lift;

    public Lift(HardwareMap hwMap) {
        lift = (ServoImplEx) hwMap.get(ServoImplEx.class, "lift_servo");
        lift.scaleRange(0.1, 0.24);
        lift.setPosition(1); // start down
    }

    public void liftUp() { lift.setPosition(0); }
    public void liftDown() { lift.setPosition(1); }

    // ---------- Actions ----------
    public Action upAction() {
        return packet -> { lift.setPosition(0); return false; };
    }

    public Action downAction() {
        return packet -> { lift.setPosition(1); return false; };
    }

    public Action liftForTime(double position, double duration) {
        return new Action() {
            private boolean init = false;
            private final com.qualcomm.robotcore.util.ElapsedTime timer = new com.qualcomm.robotcore.util.ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    timer.reset();
                    init = true;
                }
                lift.setPosition(position);
                return timer.seconds() < duration;
            }
        };
    }
}
