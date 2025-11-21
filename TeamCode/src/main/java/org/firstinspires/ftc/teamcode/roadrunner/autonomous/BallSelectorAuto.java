package org.firstinspires.ftc.teamcode.teleOp.mainOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector.BallColor;

import java.util.Arrays;

@Autonomous(name = "BallSelectorAutoTest", group = "AutoTests")
public class BallSelectorAuto extends LinearOpMode {

    private BallSelector ballSelector = new BallSelector();
    // TODO: Change this to the actual pattern according to april
    private BallColor[] targetPattern = { BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE };

    @Override
    public void runOpMode() {
        ballSelector.init(hardwareMap);

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart();

        for (BallColor target : targetPattern) {
            if (opModeIsActive()) {
                selectBall(target);
                sleep(500); // TODO: Test if this is needed
            }
        }

        telemetry.addLine("Pattern Completed");
        telemetry.update();
    }

    private void selectBall(BallColor target) {
        telemetry.addData("Target", target);
        telemetry.update();

        // 1. Spin until we see the target color at the exit
        ballSelector.spinToColor(target, telemetry);

        // 2. Push the ball
        ballSelector.pushBall();
        sleep(500); // Wait for servo to move
        ballSelector.resetPusher();
        sleep(500); // Wait for servo to reset
    }
}
