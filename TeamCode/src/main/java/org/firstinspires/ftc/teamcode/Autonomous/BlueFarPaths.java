package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueFarPaths {
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2Prep;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Park;

    public BlueFarPaths(Follower follower) {
        Shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56, 8),
                                new Pose(56, 10)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(111))
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56, 10),
                                new Pose(40, 35.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(111), -Math.toRadians(180))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(40, 35.5),
                                new Pose(56, 10)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(111))
                .build();

        Intake2Prep = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(62.230, 13.020),
                                new Pose(34.003, 35.232)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(180))
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(34.003, 35.232),
                                new Pose(15.303, 35.248)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.303, 35.248),
                                new Pose(62.368, 12.873)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
                .setReversed()
                .build();

        Intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(62.368, 12.873),
                                new Pose(7.663, 7.744)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(180))
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(7.663, 7.744),
                                new Pose(62.782, 12.452)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(62.782, 12.452),
                                new Pose(63.006, 31.364)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(90))
                .build();
    }
}
