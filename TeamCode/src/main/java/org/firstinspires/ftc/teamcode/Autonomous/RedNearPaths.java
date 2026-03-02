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
    public PathChain OpenGate;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Park;

    public RedNearPaths(Follower follower) {
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 123.000),
                                new Pose(93.500, 92.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(43))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(124.000, 127.000),
                                new Pose(74.000, 79.000),
                                new Pose(129.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        OpenGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(129.000, 83.000),
                                new Pose(119.000, 76.000),
                                new Pose(138.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(140.000, 72.000),
                                new Pose(107.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(107.000, 105.000),
                                new Pose(82.000, 54.000),
                                new Pose(128.500, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(128.500, 60.000),
                                new Pose(118.000, 55.000),
                                new Pose(107.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(107.000, 105.000),
                                new Pose(82.000, 15.000),
                                new Pose(137, 45.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(160, 30.000),
                                new Pose(107.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(107.000, 105.000),
                                new Pose(135.000, 100.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(300))
                .build();
    }
}