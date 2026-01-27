package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Indexer;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Intake;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Launcher;

@TeleOp
public class IndexerTest extends OpMode {
    private Indexer indexer;
    private Intake intake;
    private Launcher launcher;
    private MecanumDrive drive;
    private double forward, strafe, rotate, slow;

    @Override
    public void init() {
        indexer = new Indexer();
        intake = new Intake();
        launcher = new Launcher();
        drive = new MecanumDrive();

        indexer.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        drive.init(hardwareMap, telemetry);

        launcher.disableAutoPower();
    }

    @Override
    public void loop() {

        forward = MecanumDrive.smoothDrive(-1 * gamepad1.left_stick_y);
        strafe = MecanumDrive.smoothDrive(gamepad1.left_stick_x);
        rotate = MecanumDrive.smoothDrive(gamepad1.right_stick_x * 1.1);

        if (gamepad1.left_trigger > 0.4) {
            slow = 0.5;
        } else {
            slow = 1;
        }

        if (gamepad1.dpadLeftWasPressed())
            indexer.clockwise();
        else if (gamepad1.dpadRightWasPressed())
            indexer.counterClockwise();

        if (gamepad1.dpadUpWasPressed())
            launcher.stepUpPower();
        else if (gamepad1.dpadDownWasPressed())
            launcher.stepDownPower();

        if (gamepad1.triangleWasPressed())
            indexer.toggleState();

        if (gamepad1.squareWasPressed())
            intake.toggle();

        if (gamepad1.crossWasPressed())
            indexer.shoot();

        if (gamepad1.circleWasPressed())
            launcher.toggleLauncher();

        indexer.periodic();
        intake.periodic();
        launcher.updateLauncher(0, hardwareMap);
        drive.driveFieldOriented(forward, strafe, rotate, slow);

        drive.log(telemetry, slow);
        indexer.log(telemetry);
        launcher.log(telemetry);

    }
}
