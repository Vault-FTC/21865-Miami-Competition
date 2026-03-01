package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueNearGatePaths {
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Gate1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Gate2;
    public PathChain Intake3;
    public PathChain Shoot4;

    public BlueNearGatePaths(Follower follower) {
        Shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(18.282, 124.819),
                                new Pose(48.212, 95.707)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.212, 95.707),
                                new Pose(50.164, 54.281),
                                new Pose(13.896, 59.249)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();

        Gate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(13.896, 59.249),
                                new Pose(5, 74),
                                new Pose(21.230, 70.892)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.230, 70.892),
                                new Pose(47.887, 95.583)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(145))
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(47.887, 95.583),
                                new Pose(56.296, 80.444),
                                new Pose(14.269, 84.029)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();

        Gate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.269, 84.029),
                                new Pose(4.775, 49.443),
                                new Pose(48.102, 95.593)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))
                .build();

        Intake3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.102, 95.593),
                                new Pose(38.021, 46.777),
                                new Pose(4.699, 49.773)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(4.699, 49.773),
                                new Pose(57.055, 108.103)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                .build();
    }
}
