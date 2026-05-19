package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.PathMirror;

public class ShootPathBuilder {

    public static class Result {
        public final PathChain path;
        public final Pose endPose;
        public Result(PathChain path, Pose endPose) {
            this.path    = path;
            this.endPose = endPose;
        }
    }

    private final double goalX;
    private final double goalY;
    private final Follower follower;
    private final Alliance alliance;

    public ShootPathBuilder(Follower follower, Alliance alliance, double goalX, double goalY) {
        this.goalX    = goalX;
        this.goalY    = goalY;
        this.follower = follower;
        this.alliance = alliance;
    }

    public Result build(Pose... blueWaypoints) {
        Pose blueEndpoint = blueWaypoints[blueWaypoints.length - 1];
        double endHeading = Math.atan2(goalY - blueEndpoint.getY(),
                goalX - blueEndpoint.getX());

        PathBuilder builder = follower.pathBuilder();
        if (blueWaypoints.length == 2)
            builder.addPath(new BezierLine(blueWaypoints[0], blueWaypoints[1]));
        else
            builder.addPath(new BezierCurve(blueWaypoints));

        PathChain path = builder
                .setConstantHeadingInterpolation(endHeading)
                .build();

        Pose blueEndPose = new Pose(blueEndpoint.getX(), blueEndpoint.getY(), endHeading);
        if (alliance == Alliance.RED) {
            return new Result(PathMirror.flip(path), PathMirror.mirrorPose(blueEndPose));
        }
        return new Result(path, blueEndPose);
    }
}

