package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Blue-alliance near-side 6-shot intake-gate paths. */
public class NearIntakeGatePathsBlue extends NearIntakeGatePaths {

    private static final double GOAL_X = 10, GOAL_Y = 130;
    private final ShootPathBuilder shootBuilder;

    public NearIntakeGatePathsBlue(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(20, 127, Math.toRadians(140));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(20, 127), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(135)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(63, 54), new Pose(15, 63))
        ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(15, 63), new Pose(45, 70), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(70, 70), new Pose(11.5, 62.5)) //62.5, 63.5 was last
        ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160), 0.8).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(11.5, 62.5), new Pose(63, 80), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(63, 54), new Pose(11.5, 62.5))
        ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160), 0.8).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(11.5, 62.5), new Pose(63, 80), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138)).build();

        Intake4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(63, 54), new Pose(11.5, 62.5))
        ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(160), 0.8).build();

        Shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(11.5, 62.5), new Pose(63, 80), new Pose(45, 95))
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(138)).build();

        Intake5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(45, 95), new Pose(68, 83), new Pose(23, 86))
        ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180)).build();

        Shoot6 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(23, 86), new Pose(45, 105))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(146)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(45, 95), new Pose(35, 90))
        ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(270)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(20, 127), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(15, 63), new Pose(45, 70), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(14, 62.5), new Pose(63, 80), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(14, 62.5), new Pose(63, 80), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot5() { return shootBuilder.build(new Pose(14, 62.5), new Pose(63, 80), new Pose(45, 95)); }
    @Override public ShootPathBuilder.Result buildShoot6() { return shootBuilder.build(new Pose(23, 86), new Pose(50, 105)); }
}
