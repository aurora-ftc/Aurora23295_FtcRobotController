package org.firstinspires.ftc.teamcode.teleOp.mainOpModes;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Limelight;
import org.firstinspires.ftc.teamcode.teleOp.util.SmartPark;

@TeleOp(name = "DriveLaunchMode", group = "OpModes")
public class DriveLaunchMode extends OpMode {

// --- Trajectory & Drive Components ---

    private final double[] powerSteps = ConstantConfig.powerVals;
    int shotsLeft = 0;
    private final TrajectoryActionBuilder parkAction = null;
    private final MecanumDrive drive = new MecanumDrive();

// --- Timers ---
    private PinpointDrive dwive;
    private SmartPark smartPark;

// --- Timers ---

    private ElapsedTime matchTime = new ElapsedTime();
    private ElapsedTime PIDTimer = new ElapsedTime();
    private Pose2d startPose = new Pose2d(initialPoseBlue.getX(DistanceUnit.INCH),
            initialPoseBlue.getY(DistanceUnit.INCH),
            initialPoseBlue.getHeading(AngleUnit.RADIANS));
    private PinpointDrive driveRR;
    private SmartPark smartPark;
    private LaunchIntakeSystem launchSystem = new LaunchIntakeSystem();
    private BallSelector ballSelector = new BallSelector();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Pose2D initialPose, goalPose;
    private Limelight limelight = new Limelight(hardwareMap, 0);
    private double forward, strafe, rotate;
    private double lastHeading = 0;;
    private final double[] powerSteps = POWER_STEPS;
    private double slow = 1;

// --- Flags ---
    private double slow = 1;
    private boolean endgameRumbleDone, projHeadingCalculated;
    private boolean liftDown = true;
    private double startWait = 0.0;
    private double recenterTime = 0.0;

    @Override
    public void init() {

        driveRR = new PinpointDrive(hardwareMap, startPose);

        Vector2d parkPose = new Vector2d(33, -39);

        smartPark = new SmartPark(drive, driveRR, parkPose);

        // Drive Systems Init
        drive.init(hardwareMap, telemetry);

        if (BLUE_SIDE) {
            goalPose = Constants.goalPoseBlue;
        } else {
            goalPose = Constants.goalPoseRed;
        }
        drive.initTracker(goalPose, false);

        if (BLUE_SIDE) {
            initialPose = Constants.initialPoseBlue;
        } else {
            initialPose = Constants.initialPoseRed;
        }
        drive.setOdoPosition(initialPose);
        telemetry.addData("initialPose", initialPose);

        dashboard.isEnabled();

        launchSystem.init(powerSteps, hardwareMap, telemetry);
        ballSelector.init(hardwareMap);

        telemetry.addData("initialPoseGotten", drive.getOdoPosition());
        telemetry.addLine("DriveLaunchMode Initialized");
        telemetry.update();

    }

    @Override
    public void start() {
        drive.setOdoPosition(initialPose);

        lastHeading = initialPose.getHeading(AngleUnit.RADIANS);
        drive.setPIDTargetHeading(lastHeading);

        PIDTimer.reset();
        matchTime.reset();

        launchSystem.disableAutoPower();

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
                drive.drive(0, 0, 0, 0);
                telemetry.addLine("Recalibrating IMU...");
                telemetry.update();

            }
        }

        // Speed modifiers
        if (gamepad1.left_trigger > 0.4)
            slow = Constants.SLOW_SPEED_LT;
        else
            slow = 1;

        forward = MecanumDrive.smoothDrive(-gamepad1.left_stick_y);
        strafe = MecanumDrive.smoothDrive(gamepad1.left_stick_x);

        if (!drive.trackGoalOn) {
            if (Math.abs(gamepad1.right_stick_x) > 0.03) {

                rotate = MecanumDrive.smoothDrive(gamepad1.right_stick_x);
                lastHeading = drive.getOdoHeading(AngleUnit.RADIANS);
                projHeadingCalculated = false;
                PIDTimer.reset();

            } else if (PIDTimer.milliseconds() > 160) {

                if (!projHeadingCalculated) {
                    lastHeading = drive.getOdoHeading(AngleUnit.RADIANS);
                    projHeadingCalculated = true;
                }

                rotate = drive.headingPID(lastHeading);

            } else {

                rotate = 0;

            }

            drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);

        } else {

            drive.trackGoal(telemetry, forward, strafe, slow);

        }

        // Endgame rumble
        if (matchTime.seconds() >= 100 && !endgameRumbleDone) {
            gamepad1.rumble(1000);
            endgameRumbleDone = true;
        }

        // Launcher controls
        if (gamepad1.dpadUpWasPressed())
            launchSystem.stepUpPower();
        else if (gamepad1.dpadDownWasPressed())
            launchSystem.stepDownPower();

        // Launcher on/off
        if (gamepad1.triangleWasPressed())
            launchSystem.toggleLauncher();

        // Intake on/off + Reverse
        if (gamepad1.squareWasPressed())
            launchSystem.toggleIntake();
        else if (gamepad1.circleWasPressed())
            launchSystem.toggleIntakeReverse();

        // Lift hit
        if (gamepad1.dpadRightWasPressed())
            shotsLeft = 3;

        if (gamepad1.crossWasPressed() && shotsLeft == 0) {
            launchSystem.liftUp();
            startWait = matchTime.milliseconds();
            liftDown = false;
        }

        if (!liftDown && matchTime.milliseconds() >= startWait + LIFT_SERVO_FLICK_TIME) {
            launchSystem.liftDown();
            liftDown = true;
            launchSystem.intakeBlipReset();
        }

        telemetry.addData("shotsLeft", shotsLeft);
        if (shotsLeft != 0) {
            if (liftDown && matchTime.milliseconds() >= startWait + 850) {
                launchSystem.liftUp();
                startWait = matchTime.milliseconds();
                liftDown = false;
                shotsLeft--;
            }
        }

        // Reset heading
        if (gamepad1.touchpadWasPressed()) {
            drive.resetOdoHeading(telemetry);
            lastHeading = 0;
            drive.setPIDTargetHeading(lastHeading);
            gamepad1.rumbleBlips(2);
            recenterTime = matchTime.seconds();
            return;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            drive.resetOdoPosition(telemetry);
            lastHeading = 0;
            drive.setPIDTargetHeading(lastHeading);
            gamepad1.rumbleBlips(3);
            recenterTime = matchTime.seconds();
            return;
        }

        if (gamepad1.rightBumperWasPressed())
            drive.toggleTrackGoal();

        if (gamepad1.leftBumperWasPressed())
            launchSystem.toggleAutoPower();

        // Continuous subsystem updates
        double dist = drive.getDistanceFromGoal();

        launchSystem.intakeBlipLoop();
        launchSystem.updateLauncher(telemetry, dist, hardwareMap);

        // Telemetry
        if (DEBUG) {
            telemetry.addLine("Debug Enabled");
            launchSystem.debugTelemetry(telemetry);
            drive.debugTelemetry(telemetry, slow);
        } else {
            launchSystem.compTelemetry(telemetry);
            drive.updateTelemetry(telemetry, slow);
        }

        if (gamepad2.triangleWasPressed()) {
            Action parkAction = smartPark.buildParkAction();
            Actions.runBlocking(
                    new SequentialAction(
                            parkAction));
        }

        // Ball Selector Controls
        // TODO: Set to the right button
        if (gamepad1.sthWasPressed()) {
            ballSelector.spinForTime(0.5, 0.5);
        }

        if (gamepad1.sthWasPressed()) {
            ballSelector.pushBall();
            sleep(500); // TODO: See if we need something like this
            ballSelector.resetPusher();
        }

        ballSelector.telemetry(telemetry);

        telemetry.addData("BlueSide", BLUE_SIDE);

        telemetry.update();

    }
}
