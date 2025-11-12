package org.firstinspires.ftc.teamcode.roadrunner.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Lift;

@Config
@Disabled
@Autonomous(name = "AUTONOMOUS_BLUE_HBL", group = "AutoHBL")
public class BlueHBL extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-14, -62, Math.toRadians(115));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder traj1, traj2;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("-------Initialized-------");
            telemetry.addLine(">>>> Press ▶ to start. <<<<");
            telemetry.update();
        }

        //traj1 = drive.actionBuilder(initialPose).strafeTo(new Vector2d(0, 0));
        //traj2 = traj1.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(32, -39), Math.toRadians(90));
        traj1 = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-9, -24), Math.toRadians(90));
        Actions.runBlocking(
                new SequentialAction(
/*                        new ParallelAction(
                        launcher.launch(0.7, 5),
                        lift.lift(1, 2),
//                        ),
                        lift.lift(0, 2),
                        lift.lift(1, 1),
                        intake.spin(1, 1),
//                        new SleepAction(0.5),
                        intake.spin(0, 1),
                        launcher.launch(0.7, 2),
                        lift.lift(0, 2),
                        lift.lift(1, 1),
                        intake.spin(1, 1),
                        intake.spin(0, 1),
                        launcher.launch(0.7, 2),
                        lift.lift(0, 2),
                        lift.lift(1, 1),
                        traj1.build()*/
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}