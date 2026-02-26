package org.firstinspires.ftc.teamcode.Autonomous;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedNearPaths {
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake4;
    public PathChain Shoot4;

    public RedNearPaths(Follower follower) {
        Shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.268, 125.073),
                                new Pose(96.390, 96.488)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.390, 96.488),
                                new Pose(95.244, 81.512),
                                new Pose(129.756, 83.902)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(129.756, 83.902),
                                new Pose(96.171, 96.341)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.171, 96.341),
                                new Pose(90.939, 54.427),
                                new Pose(130.976, 59.098)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.976, 59.098),
                                new Pose(96.439, 96.341)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Intake4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.439, 96.341),
                                new Pose(89.463, 27.427),
                                new Pose(131.463, 35.244)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(131.463, 35.244),
                                new Pose(86.585, 114.463)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();
    }
}