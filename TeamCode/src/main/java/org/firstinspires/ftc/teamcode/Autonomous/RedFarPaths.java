package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedFarPaths {

    public double GOAL_X = 131;
    public double GOAL_Y = 143;
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Park;

    public RedFarPaths(Follower follower) {
        Shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.000, 6.000),
                                new Pose(83.000, 13.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.000, 13.000),
                                new Pose(91.582, 50),
                                new Pose(125, 43)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.645, 35.165),
                                new Pose(83.000, 13.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.000, 13.000),
                                new Pose(135, 50),
                                new Pose(130, 15)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(-90))
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(138.045, 9.620),
                                new Pose(83.000, 13.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(60))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(83.000, 13.000),
                                new Pose(117.818, 13.116)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
                .build();
    }
}
