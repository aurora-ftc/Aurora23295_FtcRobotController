package org.firstinspires.ftc.teamcode.pedroPathing.auto;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Bot3Red", group = "Bot3Autos")
@Configurable // Panels
public class Bot3Red extends OpMode {

    // Panels Telemetry
    private TelemetryManager panelsTelemetry;

    // Pedro follower
    private Follower follower;

    // State machine
    private int pathState = -1;

    // Timers
    private Timer pathTimer;
    private Timer actionTimer;

    // Subsystems
    private DcMotor launcherMotor;
    private DcMotor intakeMotor;
    private Servo liftServo;
    private Servo ts1, ts2;
    private Servo angleServo;

    // === Tunable Parameter ===

    public static double WAIT_TIME_S = 1;
    public static double TOTAL_TIME_S = 2;

    // Poses
    private final Pose startPose = new Pose(87, 8.8, Math.toRadians(0));
    private final Pose launchPose = new Pose(87, 18, Math.toRadians(0));
    private final Pose pickup1StartPose = new Pose(100, 32, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(120, 32, Math.toRadians(0));
    private final Pose pickup2StartPose = new Pose(100, 52, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(120, 52, Math.toRadians(0));
    private final Pose pickup3StartPose = new Pose(100, 78, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(120, 78, Math.toRadians(0));
    private final Pose parkPose = new Pose(85, 40, Math.toRadians(0));

    // Paths
    public PathChain firstLaunch, pickup1Start, pickup1End, secondLaunch, pickup2Start, pickup2End, thirdLaunch,
            pickup3Start, pickup3End, fourthLaunch, park;

    // === Burst scheduler ===
    private boolean burstActive = false;
    private double startTimeS = 0.0;

    // --- Build paths ---
    private void buildPaths() {
        firstLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        pickup1Start = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup1StartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickup1StartPose.getHeading())
                .build();

        pickup1End = follower.pathBuilder()
                .addPath(new BezierLine(pickup1StartPose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1StartPose.getHeading(), pickup1EndPose.getHeading())
                .build();

        secondLaunch = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, launchPose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), launchPose.getHeading())
                .build();

        pickup2Start = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup2StartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickup1StartPose.getHeading())
                .build();

        pickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickup2StartPose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), pickup2EndPose.getHeading())
                .build();

        thirdLaunch = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, launchPose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), launchPose.getHeading())
                .build();
        pickup3Start = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup3StartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickup1StartPose.getHeading())
                .build();

        pickup3End = follower.pathBuilder()
                .addPath(new BezierLine(pickup3StartPose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3StartPose.getHeading(), pickup3EndPose.getHeading())
                .build();

        fourthLaunch = follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, launchPose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), launchPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();
    }

    /** Change path/action state and reset pathTimer. */
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    // --- Burst shooting (non-blocking) ---
    private void startBurst() {
        burstActive = true;

        // Wait a bit before firing.
        startTimeS = 2;

        actionTimer.resetTimer();
    }

    /**
     * Runs the burst scheduler.
     *
     * @return true when the burst is complete (or inactive).
     */
    private boolean updateBurst() {
        if (!burstActive)
            return true;

        double t = actionTimer.getElapsedTimeSeconds();

        if ((t - startTimeS) >= WAIT_TIME_S && (t - startTimeS) < TOTAL_TIME_S) {
            liftServo.setPosition(OPEN);
            intakeMotor.setPower(1);
        }

        if ((t - startTimeS) >= TOTAL_TIME_S) {
            intakeMotor.setPower(0);
            liftServo.setPosition(CLOSE);
            burstActive = false;
            return true;
        }

        return false;
    }

    // --- Autonomous sequencing ---
    private void autonomousPathUpdate() {

        switch (pathState) {

            /*
             * 0) Start: drive firstLaunch and turn on launcher.
             */
            case 0: {
                launcherMotor.setPower(0.8);
                intakeMotor.setPower(1);
                follower.followPath(firstLaunch, 0.8, true);
                setPathState(1);
                break;
            }

            /*
             * 1) Wait until firstLaunch complete (at launchPose) and begin to execute burst
             * #1.
             */
            case 1: {
                if (!follower.isBusy()) {
                    startBurst();
                    setPathState(2);
                }
                break;
            }

            /*
             * 2) Execute burst #1 and wait until it completes, drive pickup1Start and turn
             * on intake.
             */
            case 2: {
                if (updateBurst()) {
                    follower.followPath(pickup1Start, false);
                    launcherMotor.setPower(0.7);
                    intakeMotor.setPower(1);
                    setPathState(3);
                }
                break;
            }

            /*
             * 3) Wait pickup1Start complete, then drive pickup1End.
             * During pickup1End movement, intake runs continuously.
             */
            case 3: {
                if (!follower.isBusy()) {
                    follower.followPath(pickup1End, 0.7, false);
                    setPathState(4);
                }
                break;
            }

            /*
             * 4) Wait pickup1End complete, then drive secondLaunch (back to launchPose).
             */
            case 4: {
                if (!follower.isBusy()) {
                    follower.followPath(secondLaunch, false);
                    intakeMotor.setPower(0);
                    setPathState(5);
                }
                break;
            }

            /*
             * 5) Wait until secondLaunch complete (at launchPose), then burst #2.
             */
            case 5: {
                if (!follower.isBusy()) {
                    startBurst();
                    setPathState(6);
                }
                break;
            }

            /*
             * 6) Execute burst #2 and wait until it completes, drive pickup2Start and turn
             * on intake.
             */
            case 6: {
                if (updateBurst()) {
                    follower.followPath(pickup2Start, false);
                    intakeMotor.setPower(1);
                    setPathState(7);
                }
                break;
            }

            /*
             * 7) Wait pickup2Start complete, then drive pickup2End.
             * During pickup2End movement, intake runs continuously.
             */
            case 7: {
                if (!follower.isBusy()) {
                    follower.followPath(pickup2End, 0.7, false);
                    setPathState(8);
                }
                break;
            }

            /*
             * 8) Wait pickup2End complete, then drive thirdLaunch (back to launchPose).
             */
            case 8: {
                if (!follower.isBusy()) {
                    follower.followPath(thirdLaunch, false);
                    intakeMotor.setPower(0);
                    setPathState(9);
                }
                break;
            }

            /*
             * 9) Wait until thirdLaunch complete (at launchPose), then burst #3.
             */
            case 9: {
                if (!follower.isBusy()) {
                    startBurst();
                    setPathState(10);
                }
                break;
            }

            /*
             * 10) Execute burst #3 and wait until it completes, drive pickup3Start and turn
             * on intake.
             */
            case 10: {
                if (updateBurst()) {
                    follower.followPath(pickup3Start, false);
                    intakeMotor.setPower(1);
                    setPathState(11);
                }
                break;
            }

            /*
             * 11) Wait pickup3Start complete, then drive pickup3End.
             * During pickup3End movement, intake runs continuously.
             */
            case 11: {
                if (!follower.isBusy()) {
                    follower.followPath(pickup3End, 0.7, false);
                    setPathState(12);
                }
                break;
            }

            /*
             * 12) Wait pickup3End complete, then drive fourthLaunch (back to launchPose).
             */
            case 12: {
                if (!follower.isBusy()) {
                    follower.followPath(fourthLaunch, false);
                    intakeMotor.setPower(0);
                    setPathState(13);
                }
                break;
            }

            /*
             * 13) Wait until fourthLaunch complete (at launchPose), then burst #4.
             */
            case 13: {
                if (!follower.isBusy()) {
                    startBurst();
                    setPathState(14);
                }
                break;
            }

            /*
             * 14) Execute burst #4, then drive park while turning off launcher and intake.
             */
            case 14: {
                if (updateBurst()) {
                    follower.followPath(park, false);
                    launcherMotor.setPower(0);
                    setPathState(15);
                }
                break;
            }

            /*
             * 15) Finish when park completes.
             */
            case 15: {
                if (!follower.isBusy()) {
                    launcherMotor.setPower(0);
                    setPathState(-1);
                }
                break;
            }
        }
    }

    // --- FTC OpMode lifecycle ---
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        launcherMotor = hardwareMap.get(DcMotorEx.class, HWMap.LAUNCHER_MOTOR);
        launcherMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        intakeMotor = hardwareMap.get(DcMotorEx.class, HWMap.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        liftServo = hardwareMap.get(Servo.class, HWMap.LIFT_SERVO);
        liftServo.scaleRange(0.18, 0.285);
        liftServo.setPosition(CLOSE);

        ts1 = hardwareMap.get(Servo.class, "ts1");
        ts1.setPosition(0.82);
        ts2 = hardwareMap.get(Servo.class, "ts2");
        ts2.setPosition(0.82);

        angleServo = hardwareMap.get(Servo.class, "angle_servo");
        angleServo.setPosition(0);
    }

    @Override
    public void init_loop() {
        // Optionally: pre-start localization checks, limelight init, etc.
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void stop() {
        launcherMotor.setPower(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        Pose p = follower.getPose();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.debug("X", p.getX());
        panelsTelemetry.debug("Y", p.getY());
        panelsTelemetry.debug("Heading", p.getHeading());

        panelsTelemetry.debug("BurstActive", burstActive);
        panelsTelemetry.debug("PathTimer(s)", pathTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("ActionTimer(s)", actionTimer.getElapsedTimeSeconds());

        panelsTelemetry.update(telemetry);
    }
}