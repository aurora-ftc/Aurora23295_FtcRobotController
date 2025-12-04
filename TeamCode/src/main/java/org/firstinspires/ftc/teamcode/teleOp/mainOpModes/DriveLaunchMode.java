package org.firstinspires.ftc.teamcode.teleOp.mainOpModes;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.BLUE_SIDE;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.initialPoseBlue;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallSelector;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Limelight;
import org.firstinspires.ftc.teamcode.teleOp.util.SmartPark;

@TeleOp(name = "DriveLaunchMode", group = "OpModes")
public class DriveLaunchMode extends CommandOpMode {

// --- Trajectory & Drive Components ---

    private final double[] powerSteps = Constants.POWER_STEPS;
    private final TrajectoryActionBuilder parkAction = null;
    private final MecanumDrive drive = new MecanumDrive();
    private final ElapsedTime matchTime = new ElapsedTime();
    private final ElapsedTime PIDTimer = new ElapsedTime();
    private final Pose2d startPose = new Pose2d(initialPoseBlue.getX(DistanceUnit.INCH),
            initialPoseBlue.getY(DistanceUnit.INCH),
            initialPoseBlue.getHeading(AngleUnit.RADIANS));
    private final LaunchIntakeSystem launchSystem = new LaunchIntakeSystem();
    private final BallSelector ballSelector = new BallSelector();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Limelight limelight = new Limelight(hardwareMap, 0);
    int shotsLeft = 0;
    // --- Timers ---
    private PinpointDrive dwive;
    // --- Timers ---
    private SmartPark smartPark;
    private PinpointDrive driveRR;
    private Pose2D initialPose, goalPose;
    private double forward, strafe, rotate;
    private double lastHeading = 0;
    private final double slow = 1;

    // --- Flags ---
    private boolean endgameRumbleDone, projHeadingCalculated;
    private final boolean liftDown = true;
    private final double startWait = 0.0;
    private final double recenterTime = 0.0;

    @Override
    public void initialize() {

        poseCalculated = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Drive Systems Init
        drive.init(hardwareMap, telemetry);
        launchSystem.init(POWER_STEPS, hardwareMap, telemetry);
        limelightControl = new LimelightControl(hardwareMap, llPipelines.LOCALIZATION);

        //drive.resetOdoPosition(telemetry);

        initialPose = null;

        goalPose = BLUE_SIDE ? GOAL_POSE_BLUE: GOAL_POSE_RED;

        mosaic = (Storage.mosaic == Mosaic.UNKNOWN) ? Mosaic.PGP : Storage.mosaic;
        // PGP is a lucky lucky guess, we need to have a default in case it doesn't work

        drive.initTracker(goalPose, false);

        dashboard.isEnabled();
        initTimer.reset();

    }

    @Override
    public void init_loop() {

        if (!poseCalculated && cameraTimer.milliseconds() >= 50) {
            initialPose = limelightControl.get2DLocationMT1();
            if (initialPose != null) {
                drive.setOdoPosition(initialPose);
                lastHeading = initialPose.getHeading(AngleUnit.RADIANS);
                drive.setPIDTargetHeading(lastHeading);
                poseCalculated = true;
            }
            cameraTimer.reset();
        }

        telemetry.addData("Mosaic", mosaic.name());
        telemetry.addData("initialPoseGottenX", drive.getOdoX(DistanceUnit.INCH));
        telemetry.addData("initialPoseGottenY", drive.getOdoY(DistanceUnit.INCH));
        telemetry.addData("initialPoseGottenHeading", drive.getOdoHeading(AngleUnit.DEGREES));
        telemetry.addData("initialPose", initialPose == null ? "null" : initialPose);
        telemetry.addLine("DriveLaunchMode Initialized");
        telemetry.update();

    }

    public void start() {

        if (!poseCalculated || initialPose == null){
            if (Storage.endPose == null)
                initialPose = BLUE_SIDE ? INITIAL_POSE_BLUE : INITIAL_POSE_RED;
            else
                initialPose = Storage.endPose;
        } else {
            startPose = new Pose2d(initialPose.getX(DistanceUnit.INCH),
                    initialPose.getY(DistanceUnit.INCH),
                    initialPose.getHeading(AngleUnit.RADIANS));
            driveRR = new PinpointDrive(hardwareMap, startPose);
            if (BLUE_SIDE)
                smartPark = new SmartPark(drive, driveRR, PARK_POSE_BLUE);
            else
                smartPark = new SmartPark(drive, driveRR, PARK_POSE_RED);
        }

        drive.setOdoPosition(initialPose);
        launchSystem.disableAutoPower();

        PIDTimer.reset();
        matchTime.reset();
        cameraTimer.reset();

        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();

    }

    public void run() {

    }
}
