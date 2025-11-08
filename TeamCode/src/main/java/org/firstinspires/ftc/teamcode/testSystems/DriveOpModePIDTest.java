package org.firstinspires.ftc.teamcode.testSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.ConstantConfig;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.launchSubSystem.LaunchSystem;

@Config
@TeleOp(name = "DriveOpModePIDTest", group = "TestModes")
public class DriveOpModePIDTest extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MecanumDrive drive = new MecanumDrive();
    LaunchSystem launcher = new LaunchSystem();
    ElapsedTime PIDTimer = new ElapsedTime();
    Pose2D initialPose, goalPose;
    double forward, strafe, rotate, slow;
    double lastHeading = 0;
    boolean projHeadingCalculated;
    private final double[] powerSteps = {0.6, 0.67, 0.72, 1.0};

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

        if (gamepad1.touchpadWasPressed()) {
            drive.setPIDTargetHeading(0.0);
            lastHeading = 0;
            drive.resetOdoHeading(telemetry);
        }

        if (gamepad1.rightBumperWasPressed())
            drive.toggleTrackGoal();

        if (gamepad1.dpadLeftWasPressed()) {
            drive.setPIDTargetHeading(0.0);
            lastHeading = 0;
            drive.resetOdoPosition(telemetry);
        }

        if (gamepad1.triangle) {
            launcher.spinToVelocity(0.6, ConstantConfig.flywheelKp,
                    ConstantConfig.flywheelKi, ConstantConfig.flywheelKd,
                    ConstantConfig.flywheelKv, telemetry);
        }

        telemetry.addData("rotate", rotate);
        telemetry.addData("lastHeading", lastHeading);
        telemetry.addData("Elapsed Time", getRuntime());
        telemetry.addData("goalPose", goalPose);
        telemetry.addLine();

        if (ConstantConfig.debug) {
            launcher.debugTelemetry(telemetry);
            drive.debugTelemetry(telemetry);
        } else {
            launcher.compTelemetry(telemetry);
            drive.compTelemetry(telemetry);
        }

    }
}
