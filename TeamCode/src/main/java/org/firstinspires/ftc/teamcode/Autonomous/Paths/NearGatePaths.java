package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;

public class NearGatePaths {
    private static final double START_X       = 20;
    private static final double START_Y       = 127;
    private static final double START_HEADING = 143.5;
    public double GOAL_X = 12;
    public double GOAL_Y = 135;
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Gate1V2;
    public PathChain Shoot2;
    public PathChain Shoot2Gate;
    public PathChain Intake2;
    public PathChain Gate2;
    public PathChain Shoot3Gate;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Park;
    private Pose startingPose;

    public NearGatePaths(Follower follower, Alliance alliance) {
        startingPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));

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

        Gate1V2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 86.000),
                                new Pose(39.000, 85.000),
                                new Pose(14.000, 74.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
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


        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 36.000),

                                new Pose(45.000, 95.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();


        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 95.000),
                                new Pose(35.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(250))

                .build();

        if (alliance == Alliance.RED) {
            startingPose = PathMirror.mirrorPose(startingPose);
            Shoot1      = PathMirror.flip(Shoot1);
            Intake1     = PathMirror.flip(Intake1);
            Gate1V2     = PathMirror.flip(Gate1V2);
            Shoot2      = PathMirror.flip(Shoot2);
            Shoot2Gate  = PathMirror.flip(Shoot2Gate);
            Intake2     = PathMirror.flip(Intake2);
            Gate2       = PathMirror.flip(Gate2);
            Shoot3Gate  = PathMirror.flip(Shoot3Gate);
            Shoot3      = PathMirror.flip(Shoot3);
            Intake3     = PathMirror.flip(Intake3);
            Shoot4      = PathMirror.flip(Shoot4);
            Park        = PathMirror.flip(Park);
        }
    }
    public Pose getStartingPose() {
        return startingPose;
    }
}
