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
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(135))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 54.000),
                                new Pose(15.000, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 63.000),
                                new Pose(45.000, 70.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(70.000, 70.000),
                                new Pose(18.500, 62.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160))

                .build();
        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.500, 62.000),
                                new Pose(63.000, 80.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138))

                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 54.000),
                                new Pose(18.500, 61.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160))

                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.500, 61.500),
                                new Pose(63.000, 80.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138))

                .build();

        Intake4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 54.000),
                                new Pose(18.500, 61.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160))

                .build();

        Shoot5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.500, 61.500),
                                new Pose(63.000, 80.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138))

                .build();

        Intake5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(68.000, 83.000),
                                new Pose(23.000, 86.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))

                .build();

        Shoot6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 86.000),
                                new Pose(55.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(146))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 95.000),
                                new Pose(35.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(270))

                .build();
    }
}