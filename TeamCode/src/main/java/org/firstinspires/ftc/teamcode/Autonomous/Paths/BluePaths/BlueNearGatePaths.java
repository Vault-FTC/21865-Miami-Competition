package org.firstinspires.ftc.teamcode.Autonomous.Paths.BluePaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueNearGatePaths {
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

    public BlueNearGatePaths(Follower follower) {
        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20, 127),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(137))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 54.000),
                                new Pose(15.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(45.000, 50.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(70.000, 54.000),
                                new Pose(20.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(159))

                .build();
        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.000, 63.000),
                                new Pose(23, 60),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(159), Math.toRadians(135))

                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 54.000),
                                new Pose(20.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(159))

                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.000, 63.000),
                                new Pose(45.000, 34.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(159), Math.toRadians(137))

                .build();

        Intake4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 54.000),
                                new Pose(20.000, 64.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(159))

                .build();

        Shoot5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.000, 63.000),
                                new Pose(45.000, 34.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(159), Math.toRadians(133))

                .build();

        Intake5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45, 95),
                                new Pose(23.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        Shoot6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 84.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 95.000),
                                new Pose(35.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(250))

                .build();
    }
}