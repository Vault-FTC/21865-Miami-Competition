package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;

/**
 * Far-side autonomous paths defined in Blue alliance space.
 * Coordinates can be copied directly from the PedroPathing path generator.
 * Pass Alliance.RED to mirror all paths to the Red side automatically.
 */
public class FarPaths {

    private static final double START_X       = 54;
    private static final double START_Y       = 6;
    private static final double START_HEADING = 90;

    public static final double GOAL_Y = 143;
    public final double GOAL_X; // 0 for Blue, 144 for Red

    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Park;

    private Pose startingPose;

    public FarPaths(Follower follower, Alliance alliance) {
        GOAL_X = alliance == Alliance.RED ? 144.0 : 0.0;
        startingPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));

        Shoot1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(54, 6),
                        new Pose(57, 12)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(57, 12),
                        new Pose(61.280, 33),
                        new Pose(20, 33)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180)).build();

        Shoot2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20, 33),
                        new Pose(57, 12)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57, 12),
                        new Pose(10, 5)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180)).build();

        Shoot3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(10, 10),
                        new Pose(57, 11.951)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57, 11.971),
                        new Pose(3, 13.627)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180)).build();

        Shoot4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(10, 23.627),
                        new Pose(57, 11.951)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129)).build();

        Park = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57, 11.951),
                        new Pose(57, 31.364)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(90)).build();

        if (alliance == Alliance.RED) {
            startingPose = PathMirror.mirrorPose(startingPose);
            Shoot1  = PathMirror.flip(Shoot1);
            Intake1 = PathMirror.flip(Intake1);
            Shoot2  = PathMirror.flip(Shoot2);
            Intake2 = PathMirror.flip(Intake2);
            Shoot3  = PathMirror.flip(Shoot3);
            Intake3 = PathMirror.flip(Intake3);
            Shoot4  = PathMirror.flip(Shoot4);
            Park    = PathMirror.flip(Park);
        }
    }

    public Pose getStartingPose() {
        return startingPose;
    }
}
