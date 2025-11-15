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
@Autonomous(name = "AUTONOMOUS_BLUE_SIX", group = "ChoppedAutos")
public class SixBallBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------------- Initialize ----------------------
        Pose2d initialPose = new Pose2d(63, -12, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap, telemetry);
        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        VelConstraint collectingBallsVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(8.0),
                new AngularVelConstraint(Math.toRadians(5.0))
        ));
        AccelConstraint collectingBallsAccel = new ProfileAccelConstraint(-8.0, 8.0);

        // ---------------------- Trajectories ----------------------
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(-156.5));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(55, -13, Math.toRadians(-156.5)))
                .strafeToLinearHeading(new Vector2d(36, -30), Math.toRadians(-270));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(36, -30, Math.toRadians(-270)))
                .lineToY(-54, collectingBallsVel, collectingBallsAccel);

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(36, -54, Math.toRadians(-270)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(53, -13), Math.toRadians(-156.5));
        //.splineTo(new Vector2d(53, 13), Math.toRadians(339));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(53, -13, Math.toRadians(-156.5)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(35, -13), Math.toRadians(-180));

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
                        new InstantAction(() -> launcher.farPower(hardwareMap, telemetry)),
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

                        new InstantAction(() -> launcher.farPower(hardwareMap, telemetry)),

                        tab4.build(),

                        // spin up again
                        new SleepAction(1.8), // allow time to reach velocity
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
                lift.liftForTime(0, 0.1),
                lift.liftForTime(1, 0.55),
                new SleepAction(0.2),

                new InstantAction(() -> intake.fullPower()),
                new SleepAction(0.4),
                new InstantAction(() -> intake.stop()),
                new SleepAction(0.3),

                lift.liftForTime(0, 0.1),
                lift.liftForTime(1, 0.55),
                new SleepAction(0.2),

                new InstantAction(() -> intake.fullPower()),
                new SleepAction(0.4),
                new InstantAction(() -> intake.stop()),
                new SleepAction(0.3),

                lift.liftForTime(0, 0.1),
                lift.liftForTime(1, 0.55),
                new SleepAction(0.2)
        );
    }

}
