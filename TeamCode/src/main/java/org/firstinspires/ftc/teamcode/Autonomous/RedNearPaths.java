package org.firstinspires.ftc.teamcode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedNearPaths {
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

    public RedNearPaths(Follower follower) {
        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(124.000, 127.000),
                                new Pose(81.000, 78.000),
                                new Pose(129.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                .build();

        Gate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(129.000, 84.000),
                                new Pose(119.000, 83.000),
                                new Pose(139.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(139.000, 83.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(81.000, 54.000),
                                new Pose(129.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                .build();

        Gate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(129.000, 60.000),
                                new Pose(116.000, 65.000),
                                new Pose(138.000, 80.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Shoot3Gate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(138.000, 80.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))
                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.000, 60.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(81.000, 28.000),
                                new Pose(121.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(121.000, 36.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.000, 95.000),
                                new Pose(114.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(250)) // 250 unchanged (>180)
                .build();
    }
}