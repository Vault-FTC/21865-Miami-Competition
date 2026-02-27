package org.firstinspires.ftc.teamcode.Autonomous;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueNearPaths {
    private TelemetryManager telemetryManager; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    public PathChain Prepare2ndRowArtifacts;
    public PathChain Prepare3rdRowArtifacts;
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain OpenGate;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Park;

    public BlueNearPaths(Follower follower) {
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.500, 127.000),

                                new Pose(50.500, 92.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143.6), Math.toRadians(137))

                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.500, 92.000),
                                new Pose(62.000, 79.000),
                                new Pose(15.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        OpenGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 83.000),
                                new Pose(22.558, 76.140),
                                new Pose(13.500, 70.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.500, 70.500),

                                new Pose(37.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 105.000),
                                new Pose(62.000, 54.000),
                                new Pose(17.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.000, 60.000),

                                new Pose(37.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 105.000),
                                new Pose(62.000, 27.000),
                                new Pose(15.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 36.000),

                                new Pose(37.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(37.000, 105.000),

                                new Pose(20.000, 80.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))

                .build();
    }
}