package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueFarPaths {
    public double GOAL_X = 12;
    public double GOAL_Y = 143;
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
                                new Pose(54.000, 6.000),
                                new Pose(60.000, 12.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105))
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 12.000),
                                new Pose(61.280, 37.732),
                                new Pose(20, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.122, 36.000),
                                new Pose(60.000, 12.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 12.000),
                                new Pose(20, 6)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.000, 10.000),
                                new Pose(60.024, 11.951)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
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
