package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleOp.util.DcMotorGroup;

public class Basing{

    private DcMotorEx ptoRight, ptoLeft;
    private DcMotorGroup ptoMotors;
    private boolean ascent = false;

    public void init(HardwareMap hwMap) {
        ptoLeft = hwMap.get(DcMotorEx.class, "pto_right");
        ptoRight = hwMap.get(DcMotorEx.class, "pto_left");

        ptoRight.setDirection(DcMotorEx.Direction.FORWARD);
        ptoLeft.setDirection(DcMotorEx.Direction.REVERSE); //(up is forward)

        ptoMotors = new DcMotorGroup(ptoLeft, ptoRight);

        ptoMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ptoMotors.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ptoMotors.setTolerance(0);
    }

    public void idle() {
        if (!ascent) {
            ptoMotors.setTargetPosition(0);
            ptoMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ptoMotors.setPower(-0.1);
        }
    }

    public void ascend() {
        //ptoMotors.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ascent = true;
        ptoMotors.setTargetPosition((int) (1_425.1 * 6));
        ptoMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ptoMotors.setPower(1);
    }


}
