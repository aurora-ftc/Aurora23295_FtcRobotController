package org.firstinspires.ftc.teamcode.roadrunner.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Lift;

import java.util.Arrays;

@Config
@Autonomous
public class DoubleRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(62, 13, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);


        VelConstraint collectingBallsVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(5.0),
                new AngularVelConstraint(Math.toRadians(5.0))
        ));
        AccelConstraint collectingBallsAcel = new ProfileAccelConstraint(0.0, 5.0);

        TrajectoryActionBuilder fullTab = drive.actionBuilder(initialPose)
                .lineToX(55)
                .turn(Math.toRadians(-20))
                .waitSeconds(8)
                .splineTo(new Vector2d(36, 26),Math.toRadians(200))
                .turn(Math.toRadians(70))
                .waitSeconds(1)
                .lineToY(46, collectingBallsVel, collectingBallsAcel)
                .waitSeconds(2)
                .splineTo(new Vector2d(55, 13), Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(70));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(55)
                .turn(Math.toRadians(-20));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(
                new Pose2d(55,13, Math.toRadians(160)))
                    .splineTo(new Vector2d(36, 26), Math.toRadians(200))
                    .turn(Math.toRadians(70));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(
                new Pose2d(36,26, Math.toRadians(270)))
                    .lineToY(46, collectingBallsVel, collectingBallsAcel);

        TrajectoryActionBuilder tab4 = drive.actionBuilder(
                new Pose2d(36,46, Math.toRadians(270)))
                .splineTo(new Vector2d(55, 13), Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(70));

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("-------Initialized-------");
            telemetry.addLine(">>>> Press ▶ to start. <<<<");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

//        Actions.runBlocking(
//                new SequentialAction(
//                        fullTab.build()
//                )
//        );

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                lift.lift(1, 2),
                                launcher.launch(0.7, 5),
                                tab1.build()
                        ),
                        lift.lift(0, 0.2),
                        lift.lift(1, 0.2),
                        new ParallelAction(
                                launcher.launch(0.7, 2),
                                intake.spin(1, 2)
                                ),
                        intake.spin(0, 0.8),
                        lift.lift(0, 0.2),
                        lift.lift(1, 0.2),
                        new ParallelAction(
                                launcher.launch(0.7, 2),
                                intake.spin(1, 2)
                        ),
                        intake.spin(0, 0.8),
                        lift.lift(0, 0.2),
                        lift.lift(1, 0.2),
                        launcher.launch(0, 2),
                        tab2.build(),
                        new ParallelAction(
                            intake.spin(1, 1),
                            tab3.build()
                        ),
                        intake.spin(0, 1),
                        tab4.build(),
                        lift.lift(0, 0.2),
                        lift.lift(1, 0.2),
                        launcher.launch(0.7, 5),
                        intake.spin(1, 2),
                        intake.spin(0, 0.8),
                        lift.lift(0, 0.2),
                        lift.lift(1, 0.2),
                        new ParallelAction(
                                launcher.launch(0.7, 2),
                                intake.spin(1, 2)
                        ),
                        intake.spin(0, 0.8),
                        lift.lift(0, 0.2),
                        lift.lift(1, 0.2),
                        launcher.launch(0, 2)
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}