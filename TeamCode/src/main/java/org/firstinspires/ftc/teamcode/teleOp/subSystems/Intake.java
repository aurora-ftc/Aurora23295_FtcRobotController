package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intake;
    private double power = 0;

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, HWMap.INTAKE_MOTOR);

        intake.setPower(0);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void in() {
        power = 1;
    }

    public void out() {
        power = -1;
    }

    public void off() {
        power = 0;
    }

    public void toggle() {
        if (power == 1) {
            power = 0;
        } else {
            power = 1;
        }
    }

    public void periodic() {
        intake.setPower(power);
    }
}
