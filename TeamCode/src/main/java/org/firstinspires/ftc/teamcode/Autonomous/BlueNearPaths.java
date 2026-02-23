package org.firstinspires.ftc.teamcode.Autonomous;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueNearPaths {
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

    public BlueNearPaths(Follower follower) {
//        BackUpToShootPosition = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(20.768, 123.118),
//                                new Pose(58.663, 83.522)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        PickupFirstRowArtifacts = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(58.663, 83.522),
//                                new Pose(11.676, 84.188)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        ShootPosition2 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(11.676, 84.188),
//                                new Pose(58.363, 83.505)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), -Math.toRadians(40))
//                .setReversed()
//                .build();
//
//        Prepare2ndRowArtifacts = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(58.363, 83.505),
//                                new Pose(37.176, 59.624)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
//                .build();
//
//        Pickup2ndRowArtifacts = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(37.176, 59.624),
//                                new Pose(11.957, 59.659)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        OpenGate = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(11.957, 59.659),
//                                new Pose(15.979, 69.700)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        ShootPosition3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(15.979, 69.700),
//                                new Pose(58.337, 83.403)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
//                .build();
//
//        Prepare3rdRowArtifacts = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(58.337, 83.403),
//                                new Pose(35.232, 35.232)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
//                .build();
//
//        Pickup3rdRowArtifacts = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(35.232, 35.232),
//                                new Pose(11.937, 35.401)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        FinalShootPosition = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(11.937, 35.401),
//                                new Pose(64.917, 98.280)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
//                .build();
        BackUpToShootPosition = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20.768, 123.118),
                                new Pose(50.663, 92.107)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PickupFirstRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.663, 92.107),
                                new Pose(11.676, 80.188)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        ShootPosition2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.676, 84.188),
                                new Pose(50.558, 91.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), -Math.toRadians(53))
                .setReversed()
                .build();

        Prepare2ndRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.558, 91.700),
                                new Pose(37.176, 59.624)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Pickup2ndRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(37.176, 59.624),
                                new Pose(11.957, 59.659)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        OpenGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.957, 59.659),
                                new Pose(15.979, 69.700)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        ShootPosition3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.979, 69.700),
                                new Pose(50.341, 91.512)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        Prepare3rdRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(50.341, 91.512),
                                new Pose(35.589, 35.172)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Pickup3rdRowArtifacts = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(35.589, 35.172),
                                new Pose(11.937, 35.401)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        FinalShootPosition = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.937, 35.401),
                                new Pose(60.244, 106.319)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                .build();
    }

}
