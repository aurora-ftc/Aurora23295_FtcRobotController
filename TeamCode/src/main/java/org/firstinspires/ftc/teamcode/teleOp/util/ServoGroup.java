package org.firstinspires.ftc.teamcode.teleOp.util;

import com.qualcomm.robotcore.hardware.Servo;

//Credit to Kinnqq for this classs
public class ServoGroup {
    private Servo[] servos;

    // Constructor using varargs to allow any number of motors
    public ServoGroup(Servo... servos) {
        this.servos = servos;
    }

    // Set zero power behavior for all motors
    public void setPosition(double pos) {
        for (Servo servo : servos) {
            servo.setPosition(pos);
        }
    }

    public void scaleRange(double min, double max) {
        for (Servo servo : servos) {
            servo.scaleRange(min, max);
        }
    }


}