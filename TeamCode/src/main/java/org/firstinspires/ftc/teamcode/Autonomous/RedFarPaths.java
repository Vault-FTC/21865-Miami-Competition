package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedFarPaths {
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Park;
    public RedFarPaths(Follower follower) {
        Shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 8.000),

                                new Pose(95.000, 11.000)
                        )
                ).
                setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))

                .build();

        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.000, 11.000),
                                new Pose(79.000, 38.000),
                                new Pose(130.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 35.000),

                                new Pose(95.000, 11.000)
                        )
                ).
                setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))

                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.000, 11.000),
                                new Pose(95.000, 65.000),
                                new Pose(130.000, 60.000)
                        )
                ).
                setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))

                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 60.000),

                                new Pose(95.000, 11.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))

                .build();

        Intake3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.000, 11.000),
                                new Pose(75.000, 90.000),
                                new Pose(130.000, 83.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))

                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 83.500),

                                new Pose(95.000, 11.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))

                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(95.000, 11.000),

                                new Pose(110.000, 11.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))

                .build();
    }
}