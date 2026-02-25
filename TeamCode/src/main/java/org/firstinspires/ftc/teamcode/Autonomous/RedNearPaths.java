package org.firstinspires.ftc.teamcode.Autonomous;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedNearPaths {
    private TelemetryManager telemetryManager; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    public PathChain BackUpToShootPosition;
    public PathChain PickupFirstRowArtifacts;
    public PathChain ShootPosition2;
    public PathChain Prepare2ndRowArtifacts;
    public PathChain Pickup2ndRowArtifacts;
    public PathChain OpenGate;
    public PathChain ShootPosition3;
    public PathChain Prepare3rdRowArtifacts;
    public PathChain Pickup3rdRowArtifacts;
    public PathChain FinalShootPosition;

    public RedNearPaths(Follower follower) {
        BackUpToShootPosition = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(123.232, 123.118), // 144 - 20.768
                                new Pose(93.337, 92.107)    // 144 - 50.663
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PickupFirstRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(93.337, 92.107),   // 144 - 50.663
                                new Pose(132.324, 80.188)   // 144 - 11.676
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        ShootPosition2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.324, 84.188),  // 144 - 11.676
                                new Pose(93.442, 91.700)    // 144 - 50.558
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), -Math.toRadians(53))
                .setReversed()
                .build();

        Prepare2ndRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(93.442, 91.700),   // 144 - 50.558
                                new Pose(106.824, 59.624)   // 144 - 37.176
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Pickup2ndRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(106.824, 59.624),  // 144 - 37.176
                                new Pose(132.043, 59.659)   // 144 - 11.957
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        OpenGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.043, 59.659),  // 144 - 11.957
                                new Pose(128.021, 69.700)   // 144 - 15.979
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        ShootPosition3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.021, 69.700),  // 144 - 15.979
                                new Pose(93.659, 91.512)    // 144 - 50.341
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        Prepare3rdRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(93.659, 91.512),   // 144 - 50.341
                                new Pose(108.411, 35.172)   // 144 - 35.589
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Pickup3rdRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(108.411, 35.172),  // 144 - 35.589
                                new Pose(132.063, 35.401)   // 144 - 11.937
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        FinalShootPosition = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.063, 35.401),  // 144 - 11.937
                                new Pose(83.756, 106.319)   // 144 - 60.244
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                .build();
    }
}