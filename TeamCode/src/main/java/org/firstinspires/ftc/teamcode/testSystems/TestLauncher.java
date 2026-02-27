package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teleOp.Constants;

@TeleOp
public class TestLauncher extends OpMode {
    DcMotorEx launcherM;

    @Override
    public void init() {
        launcherM = hardwareMap.get(DcMotorEx.class, "launcher_motor");
    }

    @Override
    public void loop() {
        launcherM.setPower(0.8);
    }
}
