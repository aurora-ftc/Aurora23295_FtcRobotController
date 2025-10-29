package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleOp.launchSubSystem.LaunchSystem;
@Config
@TeleOp(name = "DriveLaunchMode", group = "OpModes")
public class DriveLaunchMode extends OpMode {
    private TrajectoryActionBuilder parkAction = null;
    private MecanumDrive drive = new MecanumDrive();
    private ElapsedTime matchTime = new ElapsedTime();
    private Pose2d initialPose = new Pose2d(12, -63, Math.toRadians(90));
    private PinpointDrive dwive;
    private SmartPark smartPark;
    private LaunchSystem launchSystem = new LaunchSystem();

    private final double[] powerSteps = {0.6, 0.67, 0.72, 1.0};
    private double slow = 1;
    private boolean endgameRumbleDone;
    private boolean liftDown = true;
    private double startWait = 0.0;
    private double recenterTime = 0.0;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {

        drive.init(hardwareMap, telemetry);
        launchSystem.init(0.10, 0.24, powerSteps, hardwareMap, telemetry);

        dwive = new PinpointDrive(hardwareMap, initialPose);
        smartPark = new SmartPark(drive, dwive);

        telemetry.addLine("DriveLaunchMode Initialized");
        telemetry.update();

    }

    @Override
    public void start() {

        matchTime.reset();

        endgameRumbleDone = false;

        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();

    }

    @Override
    public void loop() {

        // Recenter freeze period (Unnecessary now but still useful)
        if (recenterTime > 0) {
            if (matchTime.seconds() - recenterTime >= 0.25) {
                recenterTime = 0;
            } else {
                drive.drive(0, 0, 0, 0, telemetry);
                telemetry.addLine("Recalibrating IMU...");
                telemetry.update();
                return;
            }
        }

        if (gamepad1.rightBumperWasPressed()) {
            Vector2d target = ConstantConfig.blueSide ?
                    new Vector2d(33.05, -37.7)
                    :new Vector2d(-33.05, -37.7);

            drive.updateOdo();
            Pose2D odoPos = drive.getOdoPosition();

            double robotX = odoPos.getX(DistanceUnit.INCH);
            double robotY = odoPos.getY(DistanceUnit.INCH);
            double currentHeading = odoPos.getHeading(AngleUnit.DEGREES);

            double angleToTarget = Math.toDegrees(Math.atan2(target.y - robotY, target.x - robotX));
            double[] cardinalAngles = {0.0, 90.0, 180.0, 270.0};
            double bestAngle = cardinalAngles[0];
            double minDiff = Math.abs(angleDiff(angleToTarget, bestAngle));

            for (double a : cardinalAngles) {
                double diff = Math.abs(angleDiff(angleToTarget, a));
                if (diff < minDiff) {
                    minDiff = diff;
                    bestAngle = a;
                }
            }

            double diffToCurrent = angleDiff(bestAngle, currentHeading);
            boolean shouldReverse = Math.abs(diffToCurrent) > 90;

            Pose2d startPose = new Pose2d(robotX, robotY, Math.toRadians(currentHeading));

            TrajectoryActionBuilder builder = dwive.actionBuilder(startPose);

            if (shouldReverse)
                builder.setReversed(true);

            builder.splineTo(target, Math.toRadians(bestAngle));

            Actions.runBlocking(new SequentialAction(parkAction.build()));
            // return Trajectory (not executed or built yet)
        }

        if (gamepad1.leftBumperWasPressed()) {
            Vector2d parkTarget = ConstantConfig.blueSide ?
                    new Vector2d(33.05, -37.7)
                    :new Vector2d(-33.05, -37.7);

            telemetry.addLine("Sent");
            parkAction = smartPark.buildParkAction(parkTarget, telemetry);
            telemetry.addLine("Returned");
            Actions.runBlocking(new SequentialAction(parkAction.build()));
            telemetry.addLine("Built");
        }

        // Driving controls
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * 1.1;

        // Speed modifiers
        if (gamepad1.left_trigger > 0.4)
            slow = 0.65;
        else if (gamepad1.right_trigger > 0.4)
            slow = 0.35;
        else slow = 1;

        // Endgame rumble
        if (matchTime.seconds() >= 100 && !endgameRumbleDone) {
            gamepad1.rumble(1000);
            endgameRumbleDone = true;
        }

        // Launcher controls
        if (gamepad1.dpad_up && !lastDpadUp)
            launchSystem.stepUpPower();
        else if (gamepad1.dpad_down && !lastDpadDown)
            launchSystem.stepDownPower();
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        //Launcher on/off
        if (gamepad1.triangleWasPressed())
            launchSystem.toggleLauncher();

        //Intake on/off + Reverse
        if (gamepad1.squareWasPressed())
            launchSystem.toggleIntake();
        if (gamepad1.circleWasPressed())
            launchSystem.toggleIntakeReverse();

        //Lift hit
        if (gamepad1.crossWasPressed()) {
            launchSystem.liftUp();
            startWait = matchTime.milliseconds();
            liftDown = false;
        }

        if (!liftDown && matchTime.milliseconds() >= startWait + 100) {
            launchSystem.liftDown();
            liftDown = true;
            launchSystem.intakeBlipReset();
        }

        //Reset heading
        if (gamepad1.touchpad) {
            gamepad1.rumbleBlips(2);
            recenterTime = matchTime.seconds();
            drive.OdoReset(telemetry);
            return;
        }

        // Continuous subsystem updates
        launchSystem.intakeBlipLoop();
        launchSystem.updateLauncher();

        drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);

        // Telemetry
        telemetry.addData("Speed", slow);
        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Rotate", rotate);

        launchSystem.updateTelemetry(telemetry);
        telemetry.update();

    }
private double angleDiff(double target, double current) {
    double diff = target - current;
    diff = ((diff + 180) % 360) - 180;
    return diff;
}
}
