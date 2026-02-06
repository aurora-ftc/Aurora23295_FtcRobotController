package org.firstinspires.ftc.teamcode.pedroPathing.auto;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Indexer;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LimelightControl;

@Autonomous(name = "Bot2Red", group = "Bot2Autos")
@Configurable // Panels
public class Bot2Red extends OpMode {

    // Panels Telemetry
    private TelemetryManager panelsTelemetry;

    // Pedro follower
    private Follower follower;

    // State machine
    private int pathState = -1;

    // Timers
    private Timer pathTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;

    // Subsystems
    private final LaunchIntakeSystem launchIntake = new LaunchIntakeSystem();
    private final Indexer indexer = new Indexer();
    private LimelightControl limelight;

    // Power steps
    private final double[] powerSteps = {0.68, 0.68};

    // === Tunable Parameter ===

    /** Must be > 0.15s because Indexer resets PUSH servo to idle after 0.15s. */
    public static double SHOT_INTERVAL_S = 0.35;

    // Poses
    private final Pose startPose = new Pose(85, 8.8, Math.toRadians(90));
    private final Pose launchPose = new Pose(85, 16,  Math.toRadians(72));
    private final Pose pickup1StartPose = new Pose(100, 35, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(120, 35, Math.toRadians(0));
    private final Pose parkPose = new Pose(85, 35,  Math.toRadians(90));

    // Paths
    public PathChain firstLaunch, pickup1Start, pickup1End, secondLaunch, park;

    // === Burst scheduler ===
    private boolean burstActive = false;
    private int shotsRemaining = 0;
    private double lastShotTimeS = 0.0;

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
        shotsRemaining = 3;
        burstActive = true;

        // Force first shot available immediately.
        lastShotTimeS = -1e9;

        actionTimer.resetTimer();

        // Refresh state before shooting.
        indexer.determineState();
    }

    /**
     * Runs the burst scheduler.
     * @return true when the burst is complete (or inactive).
     */
    private boolean updateBurst() {
        if (!burstActive) return true;

        double t = actionTimer.getElapsedTimeSeconds();

        // Indexer must run every loop for timers, gatekeeper, and PID.
        indexer.periodic();

        // Fire next shot on interval.
        if (shotsRemaining > 0 && (t - lastShotTimeS) >= SHOT_INTERVAL_S) {
            indexer.shoot();
            shotsRemaining--;
            lastShotTimeS = t;
        }

        // Done.
        if (shotsRemaining <= 0) {
            burstActive = false;
            return true;
        }

        return false;
    }

    // --- Autonomous sequencing ---
    private void autonomousPathUpdate() {

        if (!burstActive) {
            indexer.periodic();
        }

        switch (pathState) {

            /*
             * 0) Start: drive firstLaunch and turn on launcher.
             */
            case 0: {
                follower.followPath(firstLaunch);
                launchIntake.toggleLauncher();
                launchIntake.setLauncherPower(1, 0.7);
                setPathState(1);
                break;
            }

            /*
             * 1) Wait until firstLaunch complete (at launchPose) and begin to execute burst #1.
             */
            case 1: {
                if (!follower.isBusy()) {
                    startBurst();
                    setPathState(2);
                }
                break;
            }

            /*
             * 2) Execute burst #1 and wait until it completes, drive pickup1Start and turn on intake.
             */
            case 2: {
                if (updateBurst()) {
                    follower.followPath(pickup1Start, false);
                    launchIntake.toggleIntake();
                    indexer.determineState(); // refresh before intake flow
                    setPathState(3);
                }
                break;
            }

            /*
             * 3) Wait pickup1Start complete, then drive pickup1End.
             *    During pickup1End movement, intake runs continuously (indexer.periodic keeps sorting).
             */
            case 3: {
                if (!follower.isBusy()) {
                    follower.followPath(pickup1End, 0.6, false);
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
                    launchIntake.toggleIntake();
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
             * 6) Execute burst #2, then drive park while turning off launcher and intake.
             */
            case 6: {
                if (updateBurst()) {
                    follower.followPath(park, false);
                    launchIntake.toggleLauncher();
                    //launchIntake.toggleIntake();

                    setPathState(7);
                }
                break;
            }

            /*
             * 7) Finish when park completes.
             */
            case 7: {
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            }

            default:
                // idle
                break;
        }
    }

    // --- FTC OpMode lifecycle ---
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        launchIntake.init(powerSteps, hardwareMap);
        indexer.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        // Optionally: pre-start localization checks, limelight init, etc.
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
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
        panelsTelemetry.debug("ShotsRemaining", shotsRemaining);
        panelsTelemetry.debug("PathTimer(s)", pathTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("ActionTimer(s)", actionTimer.getElapsedTimeSeconds());

        panelsTelemetry.update(telemetry);
    }
}