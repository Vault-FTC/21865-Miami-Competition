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
                                new Pose(20.000, 127.000),

                                new Pose(50.500, 92.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143.6), Math.toRadians(137))

                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.000, 127.000),
                                new Pose(70.000, 79.000),
                                new Pose(15.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143.6), Math.toRadians(180))

                .build();

        OpenGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 83.000),
                                new Pose(25.000, 76.000),
                                new Pose(4.000, 76.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(4.000, 76.000),

                                new Pose(37.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 105.000),
                                new Pose(62.000, 54.000),
                                new Pose(15.576, 59.741)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

          Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.576, 59.741),
                                new Pose(26.000, 55.000),
                                new Pose(37.000, 105.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 105.000),
                                new Pose(62.000, 15.000),
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

                                new Pose(20.000, 100.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))

                .build();
    }
}