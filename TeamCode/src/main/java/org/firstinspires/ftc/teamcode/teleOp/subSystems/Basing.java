package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleOp.util.DcMotorGroup;

public class Basing{

    private DcMotorEx ptoRight, ptoLeft;
    private DcMotorGroup ptoMotors;

    public void init(HardwareMap hwMap) {
        ptoLeft = hwMap.get(DcMotorEx.class, "pto_left");
        ptoRight = hwMap.get(DcMotorEx.class, "pto_right");

        ptoRight.setDirection(DcMotorEx.Direction.FORWARD);
        ptoLeft.setDirection(DcMotorEx.Direction.REVERSE); //(up is forward)

        ptoMotors = new DcMotorGroup(ptoLeft, ptoRight);

        ptoMotors.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ptoMotors.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        ptoMotors.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void down() {
        ptoMotors.setPower(-1);
    }

    public void idle() {
        ptoMotors.setPower(0.01);
    }

    public void up() {
        ptoMotors.setPower(1);
    }

    public void ascend() {
        ptoMotors.setTargetPosition((int) (-1_425.1 * 6.7));

        ptoMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ptoMotors.setPower(1);
    }


}
