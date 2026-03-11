package org.firstinspires.ftc.teamcode.Autonomous.Paths.BluePaths;

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
    public PathChain Gate1V2;
    public PathChain Shoot2;
    public PathChain Shoot2Gate;
    public PathChain Intake2;
    public PathChain Gate2;
    public PathChain Shoot3Gate;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain GateIntake3;
    public PathChain GateShoot4;
    public PathChain Shoot4;
    public PathChain ShootToSet;
    public PathChain SetToBase;
    public PathChain BaseToShoot;
    public PathChain Park;

    public BlueNearPaths(Follower follower) {
        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20, 127),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(137))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                   new BezierCurve(
                           new Pose(45, 95),
                           new Pose(68.000, 78.000),
                           new Pose(17.000, 84.000)
                   )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        Gate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(17.000, 84.000),
                                new Pose(35.000, 83.000),
                                new Pose(9.000, 78.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Gate1V2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 86.000),
                                new Pose(39.000, 85.000),
                                new Pose(14.000, 74.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 78.000),

                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))

                .build();
        Shoot2Gate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 78.000),

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
                                new Pose(50.000, 60.000),
                                new Pose(9.000, 78.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Shoot3Gate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 78.000),

                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(23, 60),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(63.000, 28.000),
                                new Pose(25.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        GateIntake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.000, 95.000),
                                new Pose(40.000, 60.000),
                                new Pose(8.000, 62.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(157))

                .build();
        GateShoot4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.000, 62.000),
                                new Pose(45.000, 34.000),
                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(137))

                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 36.000),

                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();
        ShootToSet = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(45.000, 95.000),
                                new Pose(40.000, 10.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();

        SetToBase = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(40.000, 10.000),
                                new Pose(8.000, 9.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BaseToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.000, 9.000),
                                new Pose(45.000, 95.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
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