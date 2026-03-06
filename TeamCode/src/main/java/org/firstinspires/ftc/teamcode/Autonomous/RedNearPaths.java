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
    public PathChain Gate1V2;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Gate2;
    public PathChain Shoot3Gate;
    public PathChain Intake3Gate;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Intake3PT1;
    public PathChain Intake3PT2;
    public PathChain Intake3PT3;
    public PathChain Intake3PT4;
    public PathChain Shoot4;
    public PathChain Shoot4Intake3PT5;
    public PathChain Park;

    public RedNearPaths(Follower follower) {
        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(124.000, 127.000),
                                new Pose(81.000, 78.000),
                                new Pose(129.000, 86.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                .build();

        Gate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(129.000, 86.000),
                                new Pose(110.000, 85.000),
                                new Pose(130.000, 74.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Gate1V2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(129.000, 86.000),
                        new Pose(105.000, 85.000),
                        new Pose(130.000, 74.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130.000, 74.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(54))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(81.000, 54.000),
                                new Pose(135.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(0))
                .build();

        Gate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(135.000, 63.000),
                                new Pose(95.000, 55.000),
                                new Pose(128.000, 71.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Shoot3Gate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 71.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(55))
                .build();
        Intake3Gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(81.000, 28.000),
                                new Pose(128.000, 38.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(0))
                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(135.000, 63.000),
                                new Pose(125.000, 55.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(81.000, 28.000),
                                new Pose(128.000, 38.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                .build();

        Intake3PT1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.000, 95.000),

                                new Pose(114.000, 47.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))

                .build();

        Intake3PT2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(114.000, 47.000),

                                new Pose(130.000, 44.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Intake3PT3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130.000, 44.000),

                                new Pose(135.000, 47.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();

        Intake3PT4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(135.000, 47.000),
                                new Pose(136.000, 61.000),
                                new Pose(123.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                .build();

        Shoot4Intake3PT5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.000, 60.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(44))
                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 38.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.000, 95.000),
                                new Pose(125.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(290))
                .build();
    }
}