package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/**
 * 6-shot spike/gate near-side paths defined in Blue alliance space.
 * Coordinates can be copied directly from the PedroPathing path generator.
 * Pass Alliance.RED to mirror all paths to the Red side automatically.
 */
public class NearGateTwoPaths {

    private static final double START_X       = 20;
    private static final double START_Y       = 127;
    private static final double START_HEADING = 143.5;
    private static final double GOAL_X        = 12;
    private static final double GOAL_Y        = 135;

    public PathChain Shoot1;
    public PathChain SpikeIntake1;
    public PathChain Shoot2;
    public PathChain GateIntake1;
    public PathChain Shoot3;
    public PathChain GateIntake2;
    public PathChain Shoot4;
    public PathChain SpikeIntake2;
    public PathChain Shoot5;
    public PathChain SpikeIntake3;
    public PathChain Shoot6;

    private Pose startingPose;
    private final ShootPathBuilder shootBuilder;

    public NearGateTwoPaths(Follower follower, Alliance alliance) {
        shootBuilder = new ShootPathBuilder(follower, alliance, GOAL_X, GOAL_Y);
        startingPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(19.081, 124.005),
                        new Pose(48.126, 94.965)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(143.5)).build();

        SpikeIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(48.126, 94.965),
                        new Pose(56.515, 56.865),
                        new Pose(5.949, 59.220)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(5.949, 59.220),
                        new Pose(30.284, 52.943),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5)).build();

        GateIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(70, 70),
                        new Pose(18.5, 62)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(160)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(18.5, 62),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(143.5)).build();

        GateIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(63, 54),
                        new Pose(18.5, 62)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(160)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(18.5, 62),
                        new Pose(63, 80),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(143.5)).build();

        SpikeIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(53.286, 27.325),
                        new Pose(15, 35)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180)).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(15, 35),
                        new Pose(48.972, 57.951),
                        new Pose(45, 95)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5)).build();

        SpikeIntake3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(45, 95),
                        new Pose(38.293, 81.262),
                        new Pose(13.515, 83.696)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180)).build();

        Shoot6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(13.515, 83.696),
                        new Pose(56.847, 104.725)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150)).build();

        if (alliance == Alliance.RED) {
            startingPose    = PathMirror.mirrorPose(startingPose);
            Shoot1          = PathMirror.flip(Shoot1);
            SpikeIntake1    = PathMirror.flip(SpikeIntake1);
            Shoot2          = PathMirror.flip(Shoot2);
            GateIntake1     = PathMirror.flip(GateIntake1);
            Shoot3          = PathMirror.flip(Shoot3);
            GateIntake2     = PathMirror.flip(GateIntake2);
            Shoot4          = PathMirror.flip(Shoot4);
            SpikeIntake2    = PathMirror.flip(SpikeIntake2);
            Shoot5          = PathMirror.flip(Shoot5);
            SpikeIntake3    = PathMirror.flip(SpikeIntake3);
            Shoot6          = PathMirror.flip(Shoot6);
        }
    }

    public Pose getStartingPose() {
        return startingPose;
    }

    public ShootPathBuilder.Result buildShoot1() {
        return shootBuilder.build(new Pose(19.081, 124.005), new Pose(48.126, 94.965));
    }
    public ShootPathBuilder.Result buildShoot2() {
        return shootBuilder.build(new Pose(5.949, 59.220), new Pose(30.284, 52.943), new Pose(45, 95));
    }
    public ShootPathBuilder.Result buildShoot3() {
        return shootBuilder.build(new Pose(18.5, 62), new Pose(45, 95));
    }
    public ShootPathBuilder.Result buildShoot4() {
        return shootBuilder.build(new Pose(18.5, 62), new Pose(63, 80), new Pose(45, 95));
    }
    public ShootPathBuilder.Result buildShoot5() {
        return shootBuilder.build(new Pose(15, 35), new Pose(48.972, 57.951), new Pose(45, 95));
    }
    public ShootPathBuilder.Result buildShoot6() {
        return shootBuilder.build(new Pose(13.515, 83.696), new Pose(56.847, 104.725));
    }
}
