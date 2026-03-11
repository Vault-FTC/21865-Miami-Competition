package org.firstinspires.ftc.teamcode.Autonomous.Paths.RedPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedFarPaths {

    public double GOAL_X = 130;
    public double GOAL_Y = 143;
    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Shoot3;
    public PathChain Intake3;
    public PathChain Shoot4;
    public PathChain Park;

    public RedFarPaths(Follower follower) {

        Shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.000, 6.000),      // 144 - 54
                                new Pose(87.000, 12.000)      // 144 - 57
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),                  // 180 - 90
                        Math.toRadians(80)                   // 180 - 105
                )
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.000, 12.000),     // 144 - 57
                                new Pose(87, 45),     // 144 - 61.280
                                new Pose(130, 45)     // 144 - 20
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(80),                   // 180 - 105
                        Math.toRadians(0)                     // 180 - 180
                )
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(124.000, 45),    // 144 - 20
                                new Pose(87.000, 12.000)      // 144 - 57
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),                    // 180 - 180
                        Math.toRadians(70)                    // 180 - 120
                )
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.000, 12.000),     // 144 - 57
                                new Pose(134.000, 14)      // 144 - 10
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(70),                   // 180 - 120
                        Math.toRadians(0)                     // 180 - 180
                )
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(134.000, 14),     // 144 - 10
                                new Pose(87.000, 14)      // 144 - 57
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),                    // 180 - 180
                        Math.toRadians(70)                    // 180 - 130
                )
                .build();
        Intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(187, 11.971),
                                new Pose(134, 23.627)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(0))
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(134, 23.627),
                                new Pose(187, 11.951)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(55))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.000, 14),     // 144 - 57
                                new Pose(87.000, 31.364)      // 144 - 57
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(55),                   // 180 - 125
                        Math.toRadians(90)                    // 180 - 90
                )
                .build();
    }
}
