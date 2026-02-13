package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Constants {


    public static final double CLOSE_SHOOT_SPEED = 1100;
    public static final Pose2D BLUE_LAUNCH = new Pose2D(DistanceUnit.CM, -110, 10, AngleUnit.RADIANS, 0);
    public static final Pose2D RED_LAUNCH = new Pose2D(DistanceUnit.CM, -110, 10, AngleUnit.RADIANS, 0);
    public static final Pose2D BLUE_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -152.4, -152.4, AngleUnit.RADIANS, 0);
    public static final Pose2D RED_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -152.4, 152.4, AngleUnit.RADIANS, 0);



    public static final class Drive {
        public static final double trackEndpointHeadingMaxDistance = 12.0;
        public static final double calculateTargetHeadingMinDistance = 15.0;
        public static final double defaultFollowRadius = 15;
    }

}
