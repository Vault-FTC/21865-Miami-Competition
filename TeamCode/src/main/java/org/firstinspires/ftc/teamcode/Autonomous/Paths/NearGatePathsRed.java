package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Red-alliance near-side gate paths. Initially mirrored Blue values — tune independently. */
public class NearGatePathsRed extends NearGatePaths {

    private static final double GOAL_X = 132, GOAL_Y = 135;
    private final ShootPathBuilder shootBuilder;

    public NearGatePathsRed(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(124, 127, Math.toRadians(40));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(124, 127), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(43)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(76, 78), new Pose(127, 84))
        ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0)).build();

        Gate1V2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(127, 84), new Pose(105, 85), new Pose(132, 74))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90)).build();

        Shoot2Gate = follower.pathBuilder().addPath(
                new BezierLine(new Pose(132, 74), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(81, 54), new Pose(129, 60))
        ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0)).build();

        Gate2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(129, 60), new Pose(94, 60), new Pose(132, 74))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90)).build();

        Shoot3Gate = follower.pathBuilder().addPath(
                new BezierLine(new Pose(132, 74), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(129, 60), new Pose(121, 60), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(99, 95), new Pose(81, 28), new Pose(124, 38))
        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(119, 36), new Pose(99, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(99, 95), new Pose(109, 90))
        ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(290)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1()     { return shootBuilder.build(new Pose(124, 127), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot2Gate() { return shootBuilder.build(new Pose(132, 74), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot3Gate() { return shootBuilder.build(new Pose(132, 74), new Pose(99, 95)); }
    @Override public ShootPathBuilder.Result buildShoot4()     { return shootBuilder.build(new Pose(124, 38), new Pose(99, 95)); }
}
