package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
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
    private Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));
    private PinpointDrive dwive;
    private SmartPark smartPark;
    private LaunchSystem launchSystem = new LaunchSystem();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private ElapsedTime PIDTimer = new ElapsedTime();
    private Pose2D initialPose, goalPose;
    private double forward, strafe, rotate;
    double lastHeading = 0;
    boolean projHeadingCalculated;
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
        if (ConstantConfig.blueSide) {
            goalPose = new Pose2D(DistanceUnit.INCH, -67, 67,
                    AngleUnit.DEGREES, 0);
        } else {
            goalPose = new Pose2D(DistanceUnit.INCH, 67, 67,
                    AngleUnit.DEGREES, 0);
        }
        drive.initTracker(goalPose, false);

        dashboard.isEnabled();

        launchSystem.init(0.10, 0.24, powerSteps, hardwareMap, telemetry);

        dwive = new PinpointDrive(hardwareMap, startPose);
        smartPark = new SmartPark(drive, dwive);

        telemetry.addLine("DriveLaunchMode Initialized");
        telemetry.update();

    }

    @Override
    public void start() {

        drive.setPIDTargetHeading(Math.PI / 2.0);

        drive.resetOdoHeading(telemetry);

        drive.deactivateTrackGoal();

        if (ConstantConfig.blueSide) {
            initialPose = new Pose2D(DistanceUnit.INCH, -12, -63,
                    AngleUnit.DEGREES, 90);
        } else {
            initialPose = new Pose2D(DistanceUnit.INCH, 12, -63,
                    AngleUnit.DEGREES, 90);
        }
        drive.setOdoPosition(initialPose);

        lastHeading = initialPose.getHeading(AngleUnit.RADIANS);

        PIDTimer.reset();
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

        // Speed modifiers
        if (gamepad1.left_trigger > 0.4)
            slow = 0.65;
        else if (gamepad1.right_trigger > 0.4)
            slow = 0.35;
        else
            slow = 1;

        forward = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;

        if (!drive.trackGoalOn) {
            if (Math.abs(gamepad1.right_stick_x) > 0.03) {

                rotate = gamepad1.right_stick_x;
                lastHeading = drive.getOdoHeading(AngleUnit.RADIANS);
                projHeadingCalculated = false;
                PIDTimer.reset();

            } else if (PIDTimer.milliseconds() > 240) {

                if (!projHeadingCalculated) {
                    lastHeading = drive.getOdoHeading(AngleUnit.RADIANS);
                    projHeadingCalculated = true;
                }

                rotate = drive.headingPID(lastHeading, ConstantConfig.driveKp,
                        ConstantConfig.driveKi, ConstantConfig.driveKd);
                slow = 1;

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
            drive.setPIDTargetHeading(0.0);
            lastHeading = 0;
            gamepad1.rumbleBlips(2);
            recenterTime = matchTime.seconds();
            drive.resetOdoHeading(telemetry);
            return;
        }

        if (gamepad1.rightBumperWasPressed())
            drive.toggleTrackGoal();

        if (gamepad1.dpadLeftWasPressed()) {
            drive.setPIDTargetHeading(0.0);
            lastHeading = 0;
            drive.resetOdoPosition(telemetry);
        }

        // Continuous subsystem updates
        launchSystem.intakeBlipLoop();
        launchSystem.updateLauncher();

        //Telemetry
        if (ConstantConfig.debug) {
            launchSystem.debugTelemetry(telemetry);
            drive.debugTelemetry(telemetry);
        } else {
            launchSystem.compTelemetry(telemetry);
            drive.compTelemetry(telemetry);
        }

        telemetry.update();

    }
}
