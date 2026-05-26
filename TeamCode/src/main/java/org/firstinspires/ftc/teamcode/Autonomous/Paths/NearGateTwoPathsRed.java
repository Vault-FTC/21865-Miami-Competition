package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Red-alliance near-side 6-shot spike/gate paths. Initially mirrored Blue values — tune independently. */
public class NearGateTwoPathsRed extends NearGateTwoPaths {

    private static final double GOAL_X = 132, GOAL_Y = 135;
    private final ShootPathBuilder shootBuilder;

    public NearGateTwoPathsRed(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(124, 127, Math.toRadians(40));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(124.919, 124.005), new Pose(95.874, 94.965))
        ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(36.5)).build();

        SpikeIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(95.874, 94.965), new Pose(87.485, 56.865), new Pose(138.051, 59.220))
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(138.051, 59.220), new Pose(50, 35), new Pose(99, 99))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5)).build();

        GateIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 99), new Pose(60, 40), new Pose(138.5, 65)) //new Pose(74, 70)
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(25)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(135.5, 65.5), new Pose(87, 52.943), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(36.5)).build();

        GateIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(60, 40), new Pose(138.5, 65)) //new Pose(81, 54)
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(25)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(135.5, 65.5), new Pose(74, 63), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(36.5)).build();

        SpikeIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(90, 27), new Pose(145, 20))
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0)).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(135, 20), new Pose(95.028, 57.951), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5)).build();

        SpikeIntake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(105.707, 81.262), new Pose(130, 92))
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0)).build();

        Shoot6 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(130, 92), new Pose(110, 104.725))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(124.919, 124.005), new Pose(95.874, 94.965)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(138.051, 59.220), new Pose(113.716, 52.943), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(135.5, 64.5), new Pose(87, 52.943), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(135.5, 64.5), new Pose(81, 80), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot5() { return shootBuilder.build(new Pose(135, 38), new Pose(95.028, 57.951), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot6() { return shootBuilder.build(new Pose(130, 92), new Pose(87.153, 104.725)); }
}
