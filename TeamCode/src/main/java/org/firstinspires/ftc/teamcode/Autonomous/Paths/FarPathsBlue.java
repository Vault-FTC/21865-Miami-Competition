package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Blue-alliance far-side paths. Goal is at X=0, Y=143. */
public class FarPathsBlue extends FarPaths {

    private static final double GOAL_X = 0, GOAL_Y = 143;
    private final ShootPathBuilder shootBuilder;

    public FarPathsBlue(Follower follower) {
        shootBuilder = new ShootPathBuilder(follower, Alliance.BLUE, GOAL_X, GOAL_Y);
        startingPose = new Pose(54, 6, Math.toRadians(90));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(54, 6), new Pose(57, 12))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(57, 12), new Pose(61.280, 33), new Pose(26, 33))
        ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(20, 33), new Pose(57, 12))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(57, 12), new Pose(23, 5))
        ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(10, 10), new Pose(57, 11.951))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(57, 11.971), new Pose(23, 13.627))
        ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(10, 23.627), new Pose(57, 11.951))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(57, 11.951), new Pose(57, 31.364))
        ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(90)).build();
    }

    @Override public ShootPathBuilder.Result buildShoot1() { return shootBuilder.build(new Pose(54, 6), new Pose(57, 12)); }
    @Override public ShootPathBuilder.Result buildShoot2() { return shootBuilder.build(new Pose(20, 33), new Pose(57, 12)); }
    @Override public ShootPathBuilder.Result buildShoot3() { return shootBuilder.build(new Pose(10, 10), new Pose(57, 11.951)); }
    @Override public ShootPathBuilder.Result buildShoot4() { return shootBuilder.build(new Pose(10, 23.627), new Pose(57, 11.951)); }
}
