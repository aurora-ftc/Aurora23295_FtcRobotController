package org.firstinspires.ftc.teamcode.teleOp.mainOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleOp.ConstantConfig;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.util.SmartPark;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LimelightLocalization;

@TeleOp(name = "DriveLaunchMode", group = "OpModes")
public class DriveLaunchMode extends OpMode {
    private TrajectoryActionBuilder parkAction = null;
    private MecanumDrive drive = new MecanumDrive();
    private ElapsedTime matchTime = new ElapsedTime();
    private ElapsedTime PIDTimer = new ElapsedTime();
    private Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));
    private PinpointDrive dwive;
    private SmartPark smartPark;
    private LaunchIntakeSystem launchSystem = new LaunchIntakeSystem();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Pose2D initialPose, goalPose;
    private LimelightLocalization limelight = new LimelightLocalization();
    private double forward, strafe, rotate;
    private double lastHeading = 0;;
    private final double[] powerSteps = ConstantConfig.powerVals;
    private double slow = 1;
    private boolean endgameRumbleDone, projHeadingCalculated;
    private boolean liftDown = true;
    private double startWait = 0.0;
    private double recenterTime = 0.0;
    int shotsLeft = 0;

    @Override
    public void init() {

        dwive = new PinpointDrive(hardwareMap, startPose);
        smartPark = new SmartPark(drive, dwive);

        //Drive Systems Init
        drive.init(hardwareMap, telemetry);

        if (ConstantConfig.blueSide) {
            goalPose = ConstantConfig.goalPoseBlue;
        } else {
            goalPose = ConstantConfig.goalPoseRed;
        }
        drive.initTracker(goalPose, false);
        drive.deactivateTrackGoal();

        if (ConstantConfig.blueSide) {
            initialPose = ConstantConfig.initialPoseBlue;
        } else {
            initialPose = ConstantConfig.initialPoseRed;
        }
        drive.setOdoPosition(initialPose);
        telemetry.addData("initialPose", initialPose);

        dashboard.isEnabled();

        launchSystem.init(0.10, 0.24, powerSteps, hardwareMap, telemetry);

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
                return;
            }
        }

        // Speed modifiers
        if (gamepad1.left_trigger > 0.4) slow = 0.35;
        else slow = 1;

        forward = drive.smoothDrive(-gamepad1.left_stick_y);
        strafe = drive.smoothDrive(gamepad1.left_stick_x);

        if (!drive.trackGoalOn) {
            if (Math.abs(gamepad1.right_stick_x) > 0.03) {

                rotate = drive.smoothDrive(gamepad1.right_stick_x);
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

        } else{

            drive.trackGoal(telemetry, forward, strafe, slow);

        }

        // Endgame rumble
        if (matchTime.seconds() >= 100 && !endgameRumbleDone) {
            gamepad1.rumble(1000);
            endgameRumbleDone = true;
        }

        // Launcher controls
        if (gamepad1.dpadUpWasPressed()) launchSystem.stepUpPower();
        else if (gamepad1.dpadDownWasPressed()) launchSystem.stepDownPower();

        //Launcher on/off
        if (gamepad1.triangleWasPressed()) launchSystem.toggleLauncher();

        //Intake on/off + Reverse
        if (gamepad1.squareWasPressed()) launchSystem.toggleIntake();
        else if (gamepad1.circleWasPressed()) launchSystem.toggleIntakeReverse();

        //Lift hit
        if(gamepad1.dpadRightWasPressed()) shotsLeft = 3;


        if (gamepad1.crossWasPressed()) {
            launchSystem.liftUp();
            startWait = matchTime.milliseconds();
            liftDown = false;
        }

        if (!liftDown && matchTime.milliseconds() >= startWait + 120) {
            launchSystem.liftDown();
            liftDown = true;
            launchSystem.intakeBlipReset();
        }

        telemetry.addData("shotsLeft", shotsLeft);
        if (shotsLeft != 0) {
            if (liftDown && matchTime.milliseconds() >= startWait + 750) {
                launchSystem.liftUp();
                startWait = matchTime.milliseconds();
                liftDown = false;
                shotsLeft--;
            }
        }

        //Reset heading
        if (gamepad1.touchpadWasPressed()) {
            drive.setPIDTargetHeading(0.0);
            lastHeading = 0;
            gamepad1.rumbleBlips(2);
            recenterTime = matchTime.seconds();
            drive.resetOdoHeading(telemetry);
            return;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            drive.setPIDTargetHeading(0.0);
            lastHeading = 0;
            gamepad1.rumbleBlips(2);
            recenterTime = matchTime.seconds();
            drive.resetOdoPosition(telemetry);
            return;
        }

        if (gamepad1.rightBumperWasPressed())
            drive.toggleTrackGoal();

        if (gamepad1.leftBumperWasPressed())
            launchSystem.toggleAutoPower();


        // Continuous subsystem updates
        double dist = drive.getDistanceFromGoal();
        telemetry.addData("Distance from goal", dist);
        launchSystem.intakeBlipLoop();
        launchSystem.updateLauncher(telemetry, dist, hardwareMap);

        //Telemetry
        if (ConstantConfig.debug) {
            telemetry.addLine("Debug Enabled");
            launchSystem.debugTelemetry(telemetry);
            drive.debugTelemetry(telemetry, slow);
        } else {
            launchSystem.compTelemetry(telemetry);
            drive.compTelemetry(telemetry, slow);
        }

        telemetry.addData("BlueSide", ConstantConfig.blueSide);

        telemetry.update();

    }
}
