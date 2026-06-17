package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Red-alliance near-side 6-shot spike/gate paths. Initially mirrored Blue values — tune independently. */
public class NearGateTwoPathsRed extends NearGateTwoPaths {

    private static final double GOAL_X = 133.5, GOAL_Y = 135;
    private final ShootPathBuilder shootBuilder;

    public NearGateTwoPathsRed(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(124, 127, Math.toRadians(40));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(125, 124), new Pose(96, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(36.5)).build();

        SpikeIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(96, 95), new Pose(90, 53), new Pose(135, 62))
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(135, 62), new Pose(90, 35), new Pose(99, 99))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5)).build();

        GateIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 99), new Pose(90, 45), new Pose(136.5, 63)) //new Pose(74, 70)
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(25), 0.8).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(135, 64), new Pose(87, 53), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(36.5)).build();

        GateIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(90, 45), new Pose(136.5, 63)) //new Pose(81, 54)
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(25), 0.8).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(135, 64), new Pose(74, 63), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(36.5)).build();

        SpikeIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(90, 27), new Pose(149, 18))
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0), 0.8).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(149, 18), new Pose(95, 58), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5)).build();

        SpikeIntake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(106, 81), new Pose(133, 92))
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0)).build();

        Shoot6 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(133, 92), new Pose(90, 105))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(125, 124), new Pose(96, 95)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(135, 62), new Pose(90, 35), new Pose(99, 99)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(136.5, 63), new Pose(87, 53), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(136.5, 63), new Pose(74, 63), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot5() { return shootBuilder.build(new Pose(149, 18), new Pose(95, 58), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot6() { return shootBuilder.build(new Pose(133, 92), new Pose(90, 105)); }
}
