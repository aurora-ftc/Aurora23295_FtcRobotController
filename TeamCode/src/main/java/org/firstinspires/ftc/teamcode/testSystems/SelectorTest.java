package org.firstinspires.ftc.teamcode.testSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.POWER_STEPS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;

@TeleOp(name = "Selector Test", group = "TestModes")
public class SelectorTest extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    BallSelector selector;
    LaunchIntakeSystem launchIntakeSystem;

    @Override
    public void init() {
        selector = new BallSelector();
        launchIntakeSystem = new LaunchIntakeSystem();

        selector.init(hardwareMap);
        launchIntakeSystem.init(POWER_STEPS, hardwareMap);
        dashboard.clearTelemetry();
        dashboard.isEnabled();
    }

    @Override
    public void start() {
        selector.lampOn();
    }

    @Override
    public void loop() {
        selector.periodic();

        selector.updateTelemetry(telemetry);

        if (gamepad1.dpadLeftWasPressed()) {
            selector.moveUp();
        } else if (gamepad1.dpadRightWasPressed()) {
            selector.moveDown();
        }
        if (gamepad1.squareWasPressed()) {
            launchIntakeSystem.toggleIntake();
        }

        if (gamepad1.dpadUpWasPressed()) {

        }
    }
}
