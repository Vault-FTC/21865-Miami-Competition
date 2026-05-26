package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Blue-alliance near-side 6-shot spike/gate paths. */
public class NearGateTwoPathsBlue extends NearGateTwoPaths {

    private static final double GOAL_X = 0, GOAL_Y = 135;
    private final ShootPathBuilder shootBuilder;

    public NearGateTwoPathsBlue(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(20, 127, Math.toRadians(140));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(19.081, 124.005), new Pose(48.126, 94.965))
        ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(143.5)).build();

        SpikeIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(48.126, 94.965), new Pose(56.515, 56.865), new Pose(10, 59.220))
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(10, 59.220), new Pose(30.284, 52.943), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5)).build();

        GateIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(70, 70), new Pose(12, 63))
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(160)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(12, 63), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(143.5)).build();

        GateIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(70, 70), new Pose(12, 63)) //(new Pose(45, 95), new Pose(63, 54), new Pose(12, 63))
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(160)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(12, 63), new Pose(63, 80), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(143.5)).build();

        SpikeIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(53.286, 27.325), new Pose(15, 35))
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180)).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(15, 35), new Pose(48.972, 57.951), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5)).build();

        SpikeIntake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(38.293, 81.262), new Pose(24, 85))
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180)).build();

        Shoot6 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(24, 85), new Pose(56.847, 104.725))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(19.081, 124.005), new Pose(48.126, 94.965)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(10, 59.220), new Pose(30.284, 52.943), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(12, 63), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(12, 63), new Pose(63, 80), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot5() { return shootBuilder.build(new Pose(15, 35), new Pose(48.972, 57.951), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot6() { return shootBuilder.build(new Pose(24, 85), new Pose(56.847, 104.725)); }
}
