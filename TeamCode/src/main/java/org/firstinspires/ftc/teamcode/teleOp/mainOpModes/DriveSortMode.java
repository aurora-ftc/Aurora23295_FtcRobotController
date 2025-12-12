package org.firstinspires.ftc.teamcode.teleOp.mainOpModes;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.BLUE_SIDE;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.LIFT_SERVO_FLICK_TIME;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.initialPoseBlue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Limelight;
import org.firstinspires.ftc.teamcode.teleOp.util.SmartPark;

@TeleOp(name = "Drive Sort Mode", group = "OpModes")
public class DriveSortMode extends OpMode {

    // --- Trajectory & Drive Components ---
    private final double[] powerSteps = Constants.POWER_STEPS;
    // --- Timers ---
    private final ElapsedTime matchTime = new ElapsedTime();
    private final ElapsedTime PIDTimer = new ElapsedTime();
    private final Pose2d startPose =
            new Pose2d(initialPoseBlue.getX(DistanceUnit.INCH), initialPoseBlue.getY(DistanceUnit.INCH),
                    initialPoseBlue.getHeading(AngleUnit.RADIANS));
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private Limelight limelight;
    private MecanumDrive drive;
    private LaunchIntakeSystem launchSystem;
    private BallSelector ballSelector;
    private PinpointDrive dwive;
    private SmartPark smartPark;
    private PinpointDrive driveRR;
    private Pose2D initialPose, goalPose;
    private double forward, strafe, rotate;
    private double lastHeading = 0;
    private double slow = 1;
    // --- Flags ---
    private boolean endgameRumbleDone, projHeadingCalculated;
    private boolean liftDown = true;
    private double recenterTime = 0.0;

    @Override
    public void init() {
        drive = new MecanumDrive();
        ballSelector = new BallSelector();
        launchSystem = new LaunchIntakeSystem();
        limelight = new Limelight(hardwareMap, 0);
        driveRR = new PinpointDrive(hardwareMap, startPose);

        Vector2d parkPose = new Vector2d(33, -39);

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

        dashboard.isEnabled();

        launchSystem.init(powerSteps, hardwareMap);
        ballSelector.init(hardwareMap);

        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        multiTelemetry.addLine("━─━─━─━「₪ Drive Sort Mode Initialized ₪」━─━─━─━");
        multiTelemetry.addData("Initial Pose", initialPose);
        multiTelemetry.addData("Initial Pose (from drive)", drive.getOdoPosition());
        multiTelemetry.update();
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

        FtcDashboard.getInstance().startCameraStream(limelight.getInstance(), 30);
    }

    @Override
    public void loop() {
        // Recenter freeze period (Unnecessary now but still useful)
        if (recenterTime > 0) {
            if (matchTime.seconds() - recenterTime >= 0.25) {
                recenterTime = 0;
            } else {
                drive.drive(0, 0, 0, 0);
                MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                multiTelemetry.addLine("Recalibrating IMU...");
                multiTelemetry.update();
                return;
            }
        }

        // Speed modifiers
        if (gamepad1.left_trigger > 0.4) {
            slow = Constants.SLOW_SPEED_LT;
        } else {
            slow = 1;
        }

        forward = MecanumDrive.smoothDrive(-gamepad1.left_stick_y);
        strafe = MecanumDrive.smoothDrive(gamepad1.left_stick_x);

        //Rotate logic
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
            drive.driveFieldOriented(forward, strafe, rotate, slow);
        } else {
            drive.trackGoal(forward, strafe, slow);
        }

        // Endgame rumble
        if (matchTime.seconds() >= 100 && !endgameRumbleDone) {
            gamepad1.rumble(1000);
            endgameRumbleDone = true;
        }

        // Launcher controls
        if (gamepad1.dpadUpWasPressed()) {
            launchSystem.stepUpPower();
        } else if (gamepad1.dpadDownWasPressed()) {
            launchSystem.stepDownPower();
        }

        // Ball selector position controls
        if (gamepad1.dpadLeftWasPressed()) {
            ballSelector.moveUp();
        } else if (gamepad1.dpadRightWasPressed()) {
            ballSelector.moveDown();
        }

        // Launcher on/off
        if (gamepad1.triangleWasPressed())
            launchSystem.toggleLauncher();

        // Intake on/off + Reverse
        if (gamepad1.squareWasPressed()) {
            launchSystem.toggleIntake();
        } else if (gamepad1.circleWasPressed()) {
            launchSystem.toggleIntakeReverse();
        }

        //OBSOLETE flick reset on start
        double startWait = 0.0;
        if (!liftDown && matchTime.milliseconds() >= startWait + LIFT_SERVO_FLICK_TIME) {
            launchSystem.liftDown();
            liftDown = true;
        }

        // Reset heading
        if (gamepad1.touchpadWasPressed()) {
            drive.resetOdoHeading();
            lastHeading = 0;
            drive.setPIDTargetHeading(lastHeading);
            gamepad1.rumbleBlips(2);
            recenterTime = matchTime.seconds();
            return;
        }

        //Track goal toggle
        if (gamepad1.rightBumperWasPressed())
            drive.toggleTrackGoal();

        //Auto power toggle
        if (gamepad1.leftBumperWasPressed())
            launchSystem.toggleAutoPower();

        // Continuous subsystem updates
        double dist = drive.getDistanceFromGoal();

        launchSystem.updateLauncher(dist, hardwareMap);

        ballSelector.periodic();

        ballSelector.flashMosaicPattern();

        // Telemetry - all subsystems use updateTelemetry which sends to both Driver Station and FTC Dashboard
        if (DEBUG) {
            telemetry.addLine("!-!-!-!-! Debug Mode Enabled !-!-!-!-!");
        }
        drive.updateTelemetry(telemetry, slow);
        launchSystem.updateTelemetry(telemetry);
        ballSelector.updateTelemetry(telemetry);
        limelight.updateTelemetry(telemetry);

        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        multiTelemetry.addLine(".o0) OpMode Info (0o.");
        multiTelemetry.addData("Blue Side", BLUE_SIDE);
        multiTelemetry.update();
    }
}