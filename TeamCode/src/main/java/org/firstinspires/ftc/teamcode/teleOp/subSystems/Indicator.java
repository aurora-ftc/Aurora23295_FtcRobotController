package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.HWMap.LIGHT;

public class Indicator {
    private Servo light;

    public void init(HardwareMap hwMap) {
        light = hwMap.get(Servo.class, LIGHT);
    }

    public void off() {
        light.setPosition(0);
    }

    public void green() {
        light.setPosition(0.485);
    }

    public void red() {
        light.setPosition(0.277);
    }

    public void blue() {
        light.setPosition(0.620);
    }

    public void purple() {
        light.setPosition(0.700);
    }

    public void yellow() {
        light.setPosition(0.388);
    }
}
