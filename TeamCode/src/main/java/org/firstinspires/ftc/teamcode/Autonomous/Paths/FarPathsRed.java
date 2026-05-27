package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Red-alliance far-side paths. Initially mirrored Blue values — tune independently. */
public class FarPathsRed extends FarPaths {

    private static final double GOAL_X = 134, GOAL_Y = 143;
    private final ShootPathBuilder shootBuilder;

    public FarPathsRed(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(90, 6, Math.toRadians(90));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(90, 6), new Pose(87, 15))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(75)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(87, 12), new Pose(82.720, 33), new Pose(124, 50))
        ).setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(0)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(124, 50), new Pose(87, 15))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(87, 12), new Pose(132, 24))
        ).setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(132, 24), new Pose(87, 20))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(87, 11.971), new Pose(132, 30))
        ).setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(134, 23.627), new Pose(87, 20))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(132, 30), new Pose(87, 31.364))
        ).setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(90)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(90, 6), new Pose(87, 12)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(124, 50), new Pose(87, 12)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(132, 24), new Pose(87, 11.951)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(134, 23.627), new Pose(87, 11.951)); }
}
