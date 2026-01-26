package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.Indexer;

@TeleOp
public class IndexerTest extends OpMode {
    private Indexer indexer;

    @Override
    public void init() {
        indexer = new Indexer();
        indexer.init(hardwareMap);
    }
    @Override
    public void loop() {
        if (gamepad1.dpadLeftWasPressed())
            indexer.moveUp();
        else if (gamepad1.dpadRightWasPressed())
            indexer.moveDown();

        if (gamepad1.squareWasPressed())
            indexer.toggleState();

        indexer.updateTelemetry(telemetry);
        indexer.periodic();
    }
}
