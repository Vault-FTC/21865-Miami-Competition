package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Red-alliance far-side paths. Mirrored from Blue (x → 144 − x, heading → 180° − heading) — tune independently. */
public class FarPaths2Red extends FarPaths2 {

    private static final double GOAL_X = 136, GOAL_Y = 138;
    private final ShootPathBuilder shootBuilder;

    public FarPaths2Red(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(89.400, 8.875, Math.toRadians(90));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(89.400, 8.875), new Pose(83.111, 11.548))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60)).build();

        Spike1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(83.111, 11.548), new Pose(96.043, 37.953), new Pose(132, 50))
        ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(135, 36.061), new Pose(83.134, 11.225))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(83.134, 11.225), new Pose(108.413, 20), new Pose(134, 20))
        ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0), 0.8).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(133.152, 7.524), new Pose(83.074, 11.661))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(83.074, 11.661), new Pose(141.057, 10), new Pose(139.801, 40))
        ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(139.801, 35.869), new Pose(83.039, 11.547))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(55)).build();

        Intake4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(83.039, 11.547), new Pose(140.907, 3.441), new Pose(140.386, 50.577))
        ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90)).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(140.386, 50.577), new Pose(83.075, 11.476))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(83.075, 11.476), new Pose(87.013, 35.812))
        ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(89.400, 8.875), new Pose(83.111, 11.548)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(132, 50), new Pose(83.134, 11.225)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(133.152, 7.524), new Pose(83.074, 11.661)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(139.801, 35.869), new Pose(83.039, 11.547)); }
    @Override public ShootPathBuilder.Result buildShoot5() { return shootBuilder.build(new Pose(140.386, 50.577), new Pose(83.075, 11.476)); }
    @Override public ShootPathBuilder.Result buildShoot6() { return shootBuilder.build(new Pose(132.909, 7.544), new Pose(83.033, 11.318)); }
}
