package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ShooterConstants {

    public static final Pose2D BLUE_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -157.5, -152.4, AngleUnit.RADIANS, 0); // x, y -152.4, -152.4,
    public static final Pose2D RED_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -152.4, 152.4, AngleUnit.RADIANS, 0);
    public static double SCORE_HEIGHT = 26;
    public static double SCORE_ANGLE = Math.toRadians(-30);
    public static double PASS_THROUGH_POINT_RADIUS = 5;

    public static double HOOD_MAX_ANGLE = 70;
    public static double HOOD_MIN_ANGLE = 15;


    public static double getHoodPositionFromDegrees(double degrees) {
        return 0.15 + (degrees - 15) / (75 - 15) * (0.70 - 0.15);
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return velocity * 28 / 60.0;
    }
}
