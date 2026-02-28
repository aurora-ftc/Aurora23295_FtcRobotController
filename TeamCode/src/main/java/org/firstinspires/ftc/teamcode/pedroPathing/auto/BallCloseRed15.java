package org.firstinspires.ftc.teamcode.pedroPathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BallCloseRed15", group = "Autonomous")
@Configurable // Panels
public class BallCloseRed15 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
 // Paths defined in the Paths class
    private LaunchIntakeSystem launcher = new LaunchIntakeSystem();
    private Servo ts1, ts2, angleServo;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        launcher.init(org.firstinspires.ftc.teamcode.Constants.POWER_STEPS, hardwareMap, telemetry);

        ts1 = hardwareMap.get(Servo.class, "ts1");
        ts2 = hardwareMap.get(Servo.class, "ts2");
        angleServo = hardwareMap.get(Servo.class, "angle_servo");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121.433, 125.967, Math.toRadians(90)));

        paths(follower);

        ts2.setPosition(0.85);
        ts1.setPosition(0.85);
        angleServo.setPosition(1);

        setPathState(0);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        launcher.spinToVelocity(45, telemetry);
        launcher.updateLauncher(telemetry, 62.7, hardwareMap);

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

        public PathChain out1;
        public PathChain ball1setup;
        public PathChain ball1pickup;
        public PathChain ball1return;
        public PathChain gatePickup;
        public PathChain gateReturn;
        public PathChain ball3setup;
        public PathChain ball3pickup;
        public PathChain ball3return;
        public PathChain ball4pickup;
        public PathChain ball4return;
        public PathChain leave;

        public void paths(Follower follower) {
            out1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(121.433, 125.967),
                                    new Pose(91.883, 90.176)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                    .build();

            ball1setup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(91.883, 90.176),
                                    new Pose(99.668, 61.228)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ball1pickup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(99.668, 61.228),
                                    new Pose(123.212, 60.580)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ball1return = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(123.212, 60.580),
                                    new Pose(98.909, 61.959),
                                    new Pose(91.961, 90.296)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            gatePickup = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.961, 90.296),
                                    new Pose(112.914, 48.728),
                                    new Pose(130.954, 60.039)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            gateReturn = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.954, 60.039),
                                    new Pose(100.686, 59.752),
                                    new Pose(91.648, 89.954)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(33), Math.toRadians(0))
                    .build();

            ball3setup = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.648, 89.954),
                                    new Pose(104.508, 82.438),
                                    new Pose(128.182, 83.932)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ball3pickup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.182, 83.932),
                                    new Pose(92.358, 90.466)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ball3return = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(92.358, 90.466),
                                    new Pose(99.023, 35.221)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ball4pickup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(99.023, 35.221),
                                    new Pose(133.309, 34.980)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ball4return = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(133.309, 34.980),
                                    new Pose(92.254, 90.365)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(92.254, 90.365),
                                    new Pose(105.163, 79.537)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(52))
                    .build();
        }


    private void setPathState(int newState) {
        pathState = newState;
    }

    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0: {
                launcher.intakeOn = false;
                launcher.liftClose();
                launcher.toggleIntake();
                follower.followPath(out1, 1, true);
                setPathState(1);
                break;
            }

            /*
             * 1) Wait until firstLaunch complete (at launchPose) and begin to execute burst
             * #1.
             */
            case 1: {
                if (!follower.isBusy()) {
                    launcher.liftBlip();
                }
                break;
            }

            /*
             * 2) Execute burst #1 and wait until it completes, drive pickup1Start and turn
             * on intake.
             */
        }
    }
}