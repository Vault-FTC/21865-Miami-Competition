package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Blue-alliance near-side gate paths. */
public class NearGatePathsBlue extends NearGatePaths {

    private static final double GOAL_X = 12, GOAL_Y = 135;
    private final ShootPathBuilder shootBuilder;

    public NearGatePathsBlue(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(20, 127, Math.toRadians(140));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(20, 127), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(137)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(68, 78), new Pose(15, 84))
        ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180)).build();

        Gate1V2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(15, 84), new Pose(39, 75), new Pose(5, 70))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();

        Shoot2Gate = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(5, 70), new Pose(63, 54), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(63, 54), new Pose(15, 60))
        ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180)).build();

        Gate2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(15, 60), new Pose(50, 60), new Pose(5, 74))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();

        Shoot3Gate = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(5, 74), new Pose(63, 54), new Pose(43, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(15, 60), new Pose(23, 60), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(63, 28), new Pose(25, 36))
        ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(25, 36), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(45, 95), new Pose(35, 90))
        ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(250)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1()     { return shootBuilder.build(new Pose(20, 127), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot2Gate() { return shootBuilder.build(new Pose(5, 74), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot3Gate() { return shootBuilder.build(new Pose(5, 74), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot4()     { return shootBuilder.build(new Pose(25, 36), new Pose(45, 95)); }
}
