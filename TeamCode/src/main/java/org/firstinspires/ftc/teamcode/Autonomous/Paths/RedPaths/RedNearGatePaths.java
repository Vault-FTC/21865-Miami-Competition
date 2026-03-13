package org.firstinspires.ftc.teamcode.Autonomous.Paths.RedPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedNearGatePaths {
    public double GOAL_X = 12;
    public double GOAL_Y = 135;
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Intake4;
    public PathChain Shoot5;
    public PathChain Intake5;
    public PathChain Shoot6;
    public PathChain Park;
    public RedNearGatePaths(Follower follower) {
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 127.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(48))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(81.000, 54.000),
                                new Pose(125.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.000, 63.000),
                                new Pose(99.000, 70.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(99.000, 70.000),
                                new Pose(127.500, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(24))
                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127.500, 63.000),
                                new Pose(99.000, 80.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(24), Math.toRadians(48))
                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(81.000, 54.000),
                                new Pose(130.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(24))
                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.000, 63.000),
                                new Pose(99.000, 80.00),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(24), Math.toRadians(48))
                .build();

        Intake4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(78.000, 50.000),
                                new Pose(130.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(24))
                .build();

        Shoot5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.000, 63.000),
                                new Pose(99.000, 80.00),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(24), Math.toRadians(48))
                .build();

        Intake5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(99.000, 95.000),
                                new Pose(90.000, 83.000),
                                new Pose(127.000, 86.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                .build();

        Shoot6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.000, 86.000),
                                new Pose(99.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32))
                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.000, 95.000),
                                new Pose(125.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(290))
                .build();
    }
}
