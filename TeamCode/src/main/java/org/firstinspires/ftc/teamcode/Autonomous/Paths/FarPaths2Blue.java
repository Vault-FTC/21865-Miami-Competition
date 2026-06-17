package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Blue-alliance far-side 6-shot paths. Goal is at approximately (8, 138). */
public class FarPaths2Blue extends FarPaths2 {

    private static final double GOAL_X = 5, GOAL_Y = 130;
    private final ShootPathBuilder shootBuilder;

    public FarPaths2Blue(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(54.600, 8.875, Math.toRadians(90));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(54.600, 8.875), new Pose(60.889, 11.548))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120)).build();

        Spike1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(60.889, 11.548), new Pose(47.957, 37.953), new Pose(17, 36.061))
        ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(17, 36.061), new Pose(60.866, 11.225))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(60.866, 11.225), new Pose(35.587, 10), new Pose(10, 8))
        ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180), 0.8).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(10.848, 7.524), new Pose(60.926, 11.661))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(60.926, 11.661), new Pose(2.943, 3.618), new Pose(4.199, 35.869))
        ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(4.199, 35.869), new Pose(60.961, 11.547))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(125)).build();

        Intake4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(60.961, 11.547), new Pose(3.093, 3.441), new Pose(3.614, 50.577))
        ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90)).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(3.614, 50.577), new Pose(60.925, 11.476))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120)).build();
        Park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(60.925, 11.476), new Pose(56.987, 35.812))
        ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(54.600, 8.875), new Pose(60.889, 11.548)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(9, 36.061), new Pose(60.866, 11.225)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(10.848, 7.524), new Pose(60.926, 11.661)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(4.199, 35.869), new Pose(60.961, 11.547)); }
    @Override public ShootPathBuilder.Result buildShoot5() { return shootBuilder.build(new Pose(3.614, 50.577), new Pose(60.925, 11.476)); }
    @Override public ShootPathBuilder.Result buildShoot6() { return shootBuilder.build(new Pose(11.091, 7.544), new Pose(60.967, 11.318)); }
}
