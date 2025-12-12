package org.firstinspires.ftc.teamcode.roadrunner.autonomous;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RaceAction;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Lift;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.teleOp.util.Mosaic;

import java.util.Arrays;

@Config
@Autonomous(name = "AUTONOMOUS_BLUE_SIX", group = "ChoppedAutos")
public class SixBallBlue extends LinearOpMode {

        Mosaic mosaic = Mosaic.UNKNOWN;

        @Override
        public void runOpMode() throws InterruptedException {

                // ---------------------- Initialize ----------------------
                Pose2d initialPose = new Pose2d(63, -12, Math.toRadians(180));
                PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
                Launcher launcher = new Launcher(hardwareMap);
                Lift lift = new Lift(hardwareMap);
                Limelight limelight = new Limelight(hardwareMap, telemetry);
                Intake intake = new Intake(hardwareMap);
                ElapsedTime limelightTime = new ElapsedTime();

                VelConstraint collectingBallsVel = new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(8.0),
                                new AngularVelConstraint(Math.toRadians(5.0))));
                AccelConstraint collectingBallsAccel = new ProfileAccelConstraint(-8.0, 8.0);

                // ---------------------- Trajectories ----------------------
                TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                                .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(-156.5));

                TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(55, -13, Math.toRadians(-156.5)))
                                .strafeToLinearHeading(new Vector2d(38, -30), Math.toRadians(-270));

                TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(38, -30, Math.toRadians(-270)))
                                .lineToY(-54, collectingBallsVel, collectingBallsAccel);

                TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(38, -54, Math.toRadians(-270)))
                                .setReversed(true)
                                .strafeToLinearHeading(new Vector2d(53, -13), Math.toRadians(-156.5));

                TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(53, -13, Math.toRadians(-156.5)))
                                .setReversed(false)
                                .strafeToLinearHeading(new Vector2d(35, -13), Math.toRadians(-150));

                // ---------------------- Wait for Start ----------------------
                while (!opModeIsActive() && !isStopRequested()) {
                        if (mosaic == Mosaic.UNKNOWN && limelightTime.milliseconds() >= 50) {
                                mosaic = limelight.getLastDetectedMosaic();
                                limelightTime.reset();
                        }
                        telemetry.addData("Mosaic", mosaic.name());
                        telemetry.addLine("-------Initialized-------");
                        telemetry.addLine(">>>> Press ▶ to start. <<<<");
                        telemetry.update();
                }

                waitForStart();

                if (isStopRequested())
                        return;

                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(() -> lift.liftDown()),

                                new RaceAction(
                                        launcher.spinForTime(POWER_STEPS[0], 8),
                                        new SequentialAction(
                                                tab1.build(),
                                                new SleepAction(1.0), // Spin u// p
                                                shootThree(lift, intake, launcher, telemetry))),
                                new InstantAction(() -> launcher.stop()),

                                tab2.build(),

                                new InstantAction(() -> intake.fullPower()),
                                new SleepAction(0.5),

                                tab3.build(),

                                new InstantAction(() -> intake.stop()),
                                new SleepAction(0.5),

                                new RaceAction(
                                        launcher.spinForTime(POWER_STEPS[0], 14),
                                        new SequentialAction(
                                                tab4.build(),
                                                new SleepAction(0.8), // allow time to reach velocity
                                                shootThree(lift, intake, launcher, telemetry) // Spin up
                                        )),
                                new InstantAction(() -> launcher.stop()),

                                tab5.build()));

                telemetry.addLine("Path execution complete");
                telemetry.update();

                Storage.endPoseRR = drive.pose;

        }

    // Total time ~3.65 seconds
    private Action shootThree(Lift lift, Intake intake, Launcher launcher, Telemetry tele) {
        Action newAction = new SequentialAction(
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
                new SleepAction(0.2));
        return newAction;
    }
}
