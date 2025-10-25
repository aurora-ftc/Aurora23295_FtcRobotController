package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    // superspeed servo
    public final ServoImplEx lift;

    public Lift(HardwareMap hwMap, Telemetry telemetry) {
        lift = (ServoImplEx) hwMap.get(Servo.class, "lift_servo");
        //lift.setPwmRange(new PwmControl.PwmRange(600, 2400));
        lift.setDirection(Servo.Direction.FORWARD);
        lift.scaleRange(0.1, 0.24);
    }

    public class Flick implements Action {
        private boolean initialized = false;
//        private final ElapsedTime timer = new ElapsedTime();
//        private final double flickDuration = 0.1; // 弹上去持续时间（秒）
//        private final double totalDuration = 0.2; // 整个动作持续时间
        private final double target;

        public Flick(double target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                lift.setPosition(1);
//                //timer.reset();
//                initialized = true;
//            }

//            double elapsed = timer.seconds();

//            if (elapsed > flickDuration) {
//                lift.setPosition(1);
//            }
            lift.setPosition(target);
            telemetry.addData("target", target);

            return false;
        }
    }

    public Action lift(double target) {
        return new Flick(target);
    }
}