package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Red-alliance near-side 6-shot intake-gate paths. Initially mirrored Blue values — tune independently. */
public class NearIntakeGatePathsRed extends NearIntakeGatePaths {

    private static final double GOAL_X = 140, GOAL_Y = 140;
    private final ShootPathBuilder shootBuilder;

    public NearIntakeGatePathsRed(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(124, 127, Math.toRadians(40));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(124, 127), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(45)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(81, 54), new Pose(129, 63))
        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(129, 63), new Pose(99, 70), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(74, 70), new Pose(134.5, 66))
        ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(21)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(134.5, 66), new Pose(81, 80), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(42)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(81, 54), new Pose(134.5, 66))
        ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(21)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(134.5, 66), new Pose(81, 80), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(42)).build();

        Intake4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(81, 54), new Pose(134.5, 66))
        ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(21)).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(134.5, 66), new Pose(81, 80), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(42)).build();

        Intake5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(76, 83), new Pose(128, 86))
        ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0)).build();

        Shoot6 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(126, 86), new Pose(89, 105))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(34)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(99, 95), new Pose(109, 90))
        ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(270)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(124, 127), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(129, 63), new Pose(99, 70), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(135, 64.5), new Pose(81, 80), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(135, 64.5), new Pose(81, 80), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot5() { return shootBuilder.build(new Pose(135, 64.5), new Pose(81, 80), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot6() { return shootBuilder.build(new Pose(126, 86), new Pose(89, 105)); }
}
