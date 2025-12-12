package org.firstinspires.ftc.teamcode.testSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;

@TeleOp(name = "Selector Test", group = "TestModes")
public class SelectorTest extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //    BallSelectorNonContinuous selector;
    BallSelector selector;

    @Override
    public void init() {
//        selector = new BallSelectorNonContinuous();
        selector = new BallSelector();

        selector.init(hardwareMap);
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
    }
}
