package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotorEx intake;
    private double power = 0;
    private boolean on = false, blipper = false;
    private ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, HWMap.INTAKE_MOTOR);

        intake.setPower(0);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        timer.reset();

        off();
    }

    public void in() {
        intake.setPower(1);
    }

    public void out() {
        intake.setPower(-1);
    }

    public void off() {
        intake.setPower(0);
    }

    public void toggleIn() {
        if (blipOn()) return;

        if (on) {
            off();
            on = false;
        } else {
            in();
            on = true;
        }
    }

    public void toggleOut() {
        if (blipOn()) return;

        if (on) {
            off();
            on = false;
        } else {
            out();
            on = true;
        }
    }

    public void outtakeBlip() {
        timer.reset();
        blipper = true;
    }

    public boolean blipOn() {
        return timer.milliseconds() <= 450;
    }

    public void periodic() {
         if (blipOn())
             out();
         else if (blipper) {
             off();
             blipper = false;
         }
    }
}
