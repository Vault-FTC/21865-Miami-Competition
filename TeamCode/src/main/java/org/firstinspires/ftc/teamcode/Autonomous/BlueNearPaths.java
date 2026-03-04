package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueNearPaths {
    public double GOAL_X = 12;
    public double GOAL_Y = 135;
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Gate1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Gate2;
    public PathChain Shoot3Gate;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Park;

    public BlueNearPaths(Follower follower) {
                Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.000, 127.000),
                                new Pose(63.000, 78.000),
                                new Pose(15.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))

                .build();

        Gate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 84.000),
                                new Pose(25.000, 83.000),
                                new Pose(5.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(5.000, 83.000),

                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 54.000),
                                new Pose(15.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                .build();

        Gate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(28.000, 65.000),
                                new Pose(6.000, 80.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Shoot3Gate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(6.000, 80.000),

                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))

                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 60.000),

                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 28.000),
                                new Pose(23.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 36.000),

                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 95.000),

                                new Pose(30.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(250))

                .build();
    }
}