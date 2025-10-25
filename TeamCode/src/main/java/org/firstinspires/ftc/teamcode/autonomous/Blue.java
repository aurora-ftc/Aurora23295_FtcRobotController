package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


@Config
@Autonomous(name = "AUTONOMOUS_BLUE", group = "Auto")
public class Blue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-9, -48, Math.toRadians(115));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder traj1, traj2;

        //traj1 = drive.actionBuilder(initialPose).strafeTo(new Vector2d(0, 0));
        //traj2 = traj1.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(32, -39), Math.toRadians(90));
        traj1 = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-9, -24), Math.toRadians(90));
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                launcher.launch(0.7, 2),
                                lift.lift(0, 3)
                        ),
                        intake.spin(1, 1),
                        new SleepAction(1),
                        new ParallelAction(
                                launcher.launch(0.7, 2),
                                lift.lift(0, 3)
                        ),
                        traj1.build()
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}