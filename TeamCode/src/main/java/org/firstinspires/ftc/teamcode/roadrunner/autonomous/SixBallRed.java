package org.firstinspires.ftc.teamcode.roadrunner.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Lift;

import java.util.Arrays;

@Config
@Autonomous(name = "SixBallRed", group = "ChoppedAutos")
public class SixBallRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------------- Initialize ----------------------
        Pose2d initialPose = new Pose2d(63, 12, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        VelConstraint collectingBallsVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(5.0),
                new AngularVelConstraint(Math.toRadians(5.0))
        ));
        AccelConstraint collectingBallsAccel = new ProfileAccelConstraint(-5.0, 5.0);

        // ---------------------- Trajectories ----------------------
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(55)
                .turn(Math.toRadians(-21));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(55, 13, Math.toRadians(159)))
                .splineTo(new Vector2d(36, 26), Math.toRadians(200))
                .turn(Math.toRadians(70));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(36, 26, Math.toRadians(270)))
                .lineToY(50, collectingBallsVel, collectingBallsAccel);

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(36, 50, Math.toRadians(270)))
                .setReversed(true)
                .splineTo(new Vector2d(53, 13), Math.toRadians(339));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(53, 13, Math.toRadians(159)))
                .setReversed(false)
                .splineTo(new Vector2d(20, 13), Math.toRadians(0));

        // ---------------------- Wait for Start ----------------------
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("-------Initialized-------");
            telemetry.addLine(">>>> Press ▶ to start. <<<<");
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> lift.liftDown()),
                        new InstantAction(() -> launcher.farPower()),
                        tab1.build(),
                        new SleepAction(4.5), // Spin up
                        shootThree(lift, intake),
                        new InstantAction(() -> launcher.stop()),

                        tab2.build(),
                        new InstantAction(() -> intake.fullPower()),
                        new SleepAction(0.5),
                        tab3.build(),
                        new InstantAction(() -> intake.stop()),
                        new SleepAction(0.5),

                        tab4.build(),

                        // ⬇️ Clean, clear version of your end sequence
                        new InstantAction(() -> launcher.farPower()), // spin up again
                        new SleepAction(3.0), // allow time to reach velocity
                        shootThree(lift, intake),
                        new InstantAction(() -> launcher.stop()),

                        tab5.build()
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();

    }
    private Action shootThree(Lift lift, Intake intake) {
        return new SequentialAction(
                lift.liftForTime(0, 0.2),
                lift.liftForTime(1, 1),
                new SleepAction(0.1),

                new InstantAction(() -> intake.fullPower()),
                new SleepAction(0.8),
                new InstantAction(() -> intake.stop()),
                new SleepAction(0.7),

                lift.liftForTime(0, 0.1),
                lift.liftForTime(1, 1),
                new SleepAction(0.1),

                new InstantAction(() -> intake.fullPower()),
                new SleepAction(0.8),
                new InstantAction(() -> intake.stop()),
                new SleepAction(0.7),

                lift.liftForTime(0, 0.1),
                lift.liftForTime(1, 1),
                new SleepAction(0.1)
        );
    }

}
