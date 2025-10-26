package org.firstinspires.ftc.teamcode.roadrunner.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    //6000rpm motor
    private final DcMotorEx intakeMotor;

    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intake_motor");

        intakeMotor.setPower(0);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public class Spin implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private final double expectedTime;
        private final double powerVal;

        public Spin(double powerVal, double expectedTime) {
            this.powerVal = powerVal;
            this.expectedTime = expectedTime;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(powerVal);

            return timer.seconds() <= expectedTime;
        }

    }

    public Action spin(double powerVal, double expectedTime) {
        return new Spin(powerVal, expectedTime);
    }
}