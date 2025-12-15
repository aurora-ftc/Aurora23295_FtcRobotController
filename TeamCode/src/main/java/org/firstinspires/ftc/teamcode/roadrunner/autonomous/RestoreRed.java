package org.firstinspires.ftc.teamcode.roadrunner.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Lift;

@Config
@Autonomous(name = "AUTONOMOUS_RESTORE_RED", group = "Restore")
public class RestoreRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(9, -24, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder traj1;

        traj1 = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(14, -62), Math.toRadians(67));
        Actions.runBlocking(
                new SequentialAction(
                        traj1.build()
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}