package org.firstinspires.ftc.teamcode.Autonomous.Paths.BluePaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueNearPathsNew {
    public double GOAL_X = 12;
    public double GOAL_Y = 135;
    public PathChain MoveToShoot1;
    public PathChain SetForFirstSpike;
    public PathChain CollectFirstSpike;
    public PathChain MoveToShoot2;
    public PathChain SetSpike2;
    public PathChain CollectSpike2;
    public PathChain MoveToShoot3;
    public PathChain GateCollec1;
    public PathChain BackUpFromGate1;
    public PathChain MoveToShoot4;
    public PathChain GateCollect2;
    public PathChain BackUpFromGate2;
    public PathChain MoveToShoot5;
    public PathChain GateCollect3;
    public PathChain BackUpFromGate3;
    public PathChain MoveToShoot6;
    public PathChain GateCollect4;
    public PathChain BackUpFromGate4;
    public PathChain MoveToShoot7;
    public PathChain GateCollect5;
    public PathChain BackUpFromGate5;
    public PathChain MoveToShoot8;
    public PathChain GateCollect6;
    public PathChain BackUpFromGate6;
    public PathChain MoveToShoot9;
    public PathChain ParkPose;

    public BlueNearPathsNew(Follower follower) {
        MoveToShoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.800, 125.200),
                                new Pose(60.500, 98.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                .build();

        SetForFirstSpike = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.500, 98.700),
                                new Pose(60.500, 83.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        CollectFirstSpike = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.500, 83.500),
                                new Pose(14.000, 83.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        MoveToShoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.000, 83.700),
                                new Pose(60.500, 98.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();

        SetSpike2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.500, 98.700),
                                new Pose(48.300, 60.200)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        CollectSpike2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(48.300, 60.200),
                                new Pose(10.000, 60.200)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        MoveToShoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 60.200),
                                new Pose(25.000, 60.200)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        GateCollec1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(25.000, 60.200),
                                new Pose(52.000, 69.000),
                                new Pose(60.500, 98.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();

        BackUpFromGate1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.500, 98.700),
                                new Pose(41.000, 67.500),
                                new Pose(8.300, 60.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                .build();

        MoveToShoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.300, 60.500),
                                new Pose(25.000, 60.200)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        GateCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(25.000, 60.200),
                                new Pose(52.000, 69.000),
                                new Pose(60.500, 98.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(143))
                .build();

        BackUpFromGate2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.500, 98.700),
                                new Pose(41.000, 67.500),
                                new Pose(8.300, 60.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                .build();

        MoveToShoot5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.300, 60.500),
                                new Pose(25.000, 60.200)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        GateCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(25.000, 60.200),
                                new Pose(52.000, 69.000),
                                new Pose(60.500, 98.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(143))
                .build();

        BackUpFromGate3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.500, 98.700),
                                new Pose(41.000, 67.500),
                                new Pose(8.300, 60.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                .build();

        MoveToShoot6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.300, 60.500),
                                new Pose(25.000, 60.200)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        GateCollect4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(25.000, 60.200),
                                new Pose(52.000, 69.000),
                                new Pose(60.500, 98.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(143))
                .build();

        BackUpFromGate4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.500, 98.700),
                                new Pose(41.000, 67.500),
                                new Pose(8.300, 60.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                .build();

        MoveToShoot7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.300, 60.500),
                                new Pose(25.000, 60.200)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        GateCollect5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(25.000, 60.200),
                                new Pose(52.000, 69.000),
                                new Pose(60.500, 98.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(143))
                .build();

        BackUpFromGate5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.500, 98.700),
                                new Pose(41.000, 67.500),
                                new Pose(8.300, 60.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                .build();

        MoveToShoot8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.300, 60.500),
                                new Pose(25.000, 60.200)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        GateCollect6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(25.000, 60.200),
                                new Pose(52.000, 69.000),
                                new Pose(60.500, 98.700)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(143))
                .build();

        BackUpFromGate6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.500, 98.700),
                                new Pose(30.000, 77.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(270))
                .build();
    }
}



