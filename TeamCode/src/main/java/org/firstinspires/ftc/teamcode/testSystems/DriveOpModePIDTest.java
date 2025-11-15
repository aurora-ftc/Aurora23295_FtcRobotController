package org.firstinspires.ftc.teamcode.testSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleOp.util.ConstantConfig;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;

@Config
@TeleOp(name = "DriveOpModePIDTest", group = "TestModes")
public class DriveOpModePIDTest extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MecanumDrive drive = new MecanumDrive();
    LaunchIntakeSystem launcher = new LaunchIntakeSystem();
    ElapsedTime PIDTimer = new ElapsedTime();
    Pose2D initialPose, goalPose;
    double forward, strafe, rotate, slow;
    double startWait;
    double lastHeading = 0;
    boolean projHeadingCalculated;
    private final double[] powerSteps = {50, 57, 62, 85};
    boolean shooterOn, liftDown;
    private ElapsedTime matchTime = new ElapsedTime();

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

        launcher.init(0.1,0.24, powerSteps, hardwareMap, telemetry);

        shooterOn = false;

    }

    @Override
    public void start() {

        drive.setPIDTargetHeading(Math.PI / 2.0);

        drive.resetOdoHeading();

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

        matchTime.reset();

    }

    @Override
    public void loop() {

        if (gamepad1.left_trigger > 0.4) {
            slow = 0.35;
        } else if (gamepad1.right_trigger > 0.4) {
            slow = 0.65;
        } else {
            slow = 1;
        }

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

                rotate = drive.headingPID(lastHeading);
                slow = 1;

            } else {

                rotate = 0;

            }

            drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);

        } else{

            drive.trackGoal(telemetry, forward, strafe, slow);

        }

        if (gamepad1.touchpadWasPressed()) {
            drive.setPIDTargetHeading(0.0);
            lastHeading = 0;
            drive.resetOdoHeading();
        }

        if (gamepad1.rightBumperWasPressed())
            drive.toggleTrackGoal();

        if (gamepad1.dpadLeftWasPressed()) {
            drive.setPIDTargetHeading(0.0);
            lastHeading = 0;
            drive.resetOdoPosition(telemetry);
        }

        if (gamepad1.triangleWasPressed()) {
            launcher.toggleLauncher();
        }

        if (gamepad1.squareWasPressed())
            launcher.toggleIntake();
        if (gamepad1.circleWasPressed())
            launcher.toggleIntakeReverse();

        if (gamepad1.dpadUpWasPressed())
            launcher.stepUpPower();
        else if (gamepad1.dpadDownWasPressed())
            launcher.stepDownPower();

        if (gamepad1.crossWasPressed()) {
            launcher.liftUp();
            startWait = matchTime.milliseconds();
            liftDown = false;
        }

        if (!liftDown && matchTime.milliseconds() >= startWait + 100) {
            launcher.liftDown();
            liftDown = true;
            launcher.intakeBlipReset();
        }

        double dist = drive.getDistanceFromGoal();
        launcher.intakeBlipLoop();
        launcher.updateLauncher(telemetry, dist, hardwareMap);

        telemetry.addData("rotate", rotate);
        telemetry.addData("lastHeading", lastHeading);
        telemetry.addData("Elapsed Time", getRuntime());
        telemetry.addData("goalPose", goalPose);
        telemetry.addLine();

        if (ConstantConfig.debug) {
            launcher.debugTelemetry(telemetry);
            drive.debugTelemetry(telemetry, slow);
        } else {
            launcher.compTelemetry(telemetry);
            drive.updateTelemetry(telemetry, slow);
        }

    }
}
