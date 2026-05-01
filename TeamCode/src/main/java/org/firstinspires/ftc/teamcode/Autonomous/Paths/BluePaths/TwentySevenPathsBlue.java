package org.firstinspires.ftc.teamcode.Autonomous.Paths.BluePaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TwentySevenPathsBlue {
        public static PathChain MoveToShoot1;
        public static PathChain SetFirstSpike;
        public static PathChain CollectFirstSpike;
    public PathChain MoveToShoot2;
    public PathChain moveaway;

        public TwentySevenPathsBlue(Follower follower) {
            MoveToShoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20, 127),
                                    new Pose(53, 100)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(143.5))
                    .build();

            SetFirstSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53, 100),
                                    new Pose(43, 84)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))
                    .build();

            CollectFirstSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(43, 84),
                                    new Pose(10.5, 84)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            MoveToShoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.5, 84),
                                    new Pose(53, 100)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5))
                    .build();

            moveaway = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53, 100),
                                    new Pose(50, 50)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
}


