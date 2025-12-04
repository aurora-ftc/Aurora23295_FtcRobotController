package org.firstinspires.ftc.teamcode.teleOp.mainOpModes;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.BLUE_SIDE;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.initialPoseBlue;

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
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
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

    public void run() {

    }
}
