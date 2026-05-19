package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;

/**
 * 6-shot intake-gate near-side paths defined in Blue alliance space.
 * Coordinates can be copied directly from the PedroPathing path generator.
 * Pass Alliance.RED to mirror all paths to the Red side automatically.
 */
public class NearIntakeGatePaths {

    private static final double START_X       = 20;
    private static final double START_Y       = 127;
    private static final double START_HEADING = 143.5;

    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Intake4;
    public PathChain Shoot5;
    public PathChain Intake5;
    public PathChain Shoot6;
    public PathChain Park;

    private Pose startingPose;

    public NearIntakeGatePaths(Follower follower, Alliance alliance) {
        startingPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20, 127),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(135)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(63, 54),
                        new Pose(15, 63)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(15, 63),
                        new Pose(45, 70),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(70, 70),
                        new Pose(18.5, 62)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(18.5, 62),
                        new Pose(63, 80),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(63, 54),
                        new Pose(18.5, 61.5)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(18.5, 61.5),
                        new Pose(63, 80),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138)).build();

        Intake4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(63, 54),
                        new Pose(18.5, 61.5)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160)).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(18.5, 61.5),
                        new Pose(63, 80),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138)).build();

        Intake5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(68, 83),
                        new Pose(23, 86)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180)).build();

        Shoot6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(23, 86),
                        new Pose(55, 105)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(146)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(45, 95),
                        new Pose(35, 90)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(270)).build();

        if (alliance == Alliance.RED) {
            startingPose = PathMirror.mirrorPose(startingPose);
            Shoot1  = PathMirror.flip(Shoot1);
            Intake1 = PathMirror.flip(Intake1);
            Shoot2  = PathMirror.flip(Shoot2);
            Intake2 = PathMirror.flip(Intake2);
            Shoot3  = PathMirror.flip(Shoot3);
            Intake3 = PathMirror.flip(Intake3);
            Shoot4  = PathMirror.flip(Shoot4);
            Intake4 = PathMirror.flip(Intake4);
            Shoot5  = PathMirror.flip(Shoot5);
            Intake5 = PathMirror.flip(Intake5);
            Shoot6  = PathMirror.flip(Shoot6);
            Park    = PathMirror.flip(Park);
        }
    }

    public Pose getStartingPose() {
        return startingPose;
    }
}
