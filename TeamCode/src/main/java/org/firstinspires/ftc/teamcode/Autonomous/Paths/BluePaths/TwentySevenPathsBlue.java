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
    public static PathChain MoveToShoot2;
    public static PathChain SetSecondSpike;
    public static PathChain CollectSecondSpike;
    public static PathChain MoveToShoot3;


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
                                    new Pose(43, 86)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))
                    .build();

            CollectFirstSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(43, 86),
                                    new Pose(15, 86)
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


            SetSecondSpike = follower.pathBuilder()
                                    .addPath(new BezierLine(
                    new Pose(53.000, 100.000),
                    new Pose(47.500, 60.500))
                    )

             .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            CollectSecondSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(47.500, 60.500),
                                    new Pose(8.000, 60.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            MoveToShoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.000, 60.500),
                                    new Pose(47.500, 60.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            MoveToShoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(47.500, 60.500),
                                    new Pose(53.000, 100.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5))
                    .build();

        }
}