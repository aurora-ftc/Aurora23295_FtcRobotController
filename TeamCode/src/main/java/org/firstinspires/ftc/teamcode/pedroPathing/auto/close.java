package org.firstinspires.ftc.teamcode.pedroPathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class close extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain OutFirst;
        public PathChain BallsClose;
        public PathChain CloseReturn;
        public PathChain MidSetup;
        public PathChain BallsMid;
        public PathChain MidReturn;
        public PathChain FarSetup;
        public PathChain BallsFar;
        public PathChain FarReurn;
        public PathChain Leave;

        public Paths(Follower follower) {
            OutFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(120.938, 126.801),
                                    new Pose(92.169, 92.169)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(48))
                    .build();

            BallsClose = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.169, 92.169),
                                    new Pose(97.933, 82.419),
                                    new Pose(129.593, 83.469)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            CloseReturn = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(129.593, 83.469),
                                    new Pose(92.059, 92.736)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                    .build();

            MidSetup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(92.059, 92.736),
                                    new Pose(99.590, 58.104)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                    .build();

            BallsMid = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(99.590, 58.104),
                                    new Pose(133.759, 57.948)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            MidReturn = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.759, 57.948),
                                    new Pose(108.267, 59.093),
                                    new Pose(97.355, 69.191),
                                    new Pose(92.847, 92.381)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                    .build();

            FarSetup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(92.847, 92.381),
                                    new Pose(101.049, 35.554)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                    .build();

            BallsFar = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(101.049, 35.554),
                                    new Pose(133.156, 35.785)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            FarReurn = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.156, 35.785),
                                    new Pose(107.228, 51.852),
                                    new Pose(92.583, 92.336)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(92.583, 92.336),
                                    new Pose(103.107, 78.104)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(53))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return 0;
    }
}


