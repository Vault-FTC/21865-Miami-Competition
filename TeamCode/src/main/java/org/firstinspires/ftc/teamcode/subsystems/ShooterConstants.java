package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ShooterConstants {

    public static final Pose2D BLUE_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -157.5, -152.4, AngleUnit.RADIANS, 0); // x, y -152.4, -152.4,
    public static final Pose2D RED_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -152.4, 152.4, AngleUnit.RADIANS, 0);
    public static double SCORE_HEIGHT = 76.5;
    public static double SCORE_ANGLE = Math.toRadians(-30);
    public static double PASS_THROUGH_POINT_RADIUS = 5;

    public static double HOOD_MAX_ANGLE = 70;
    public static double HOOD_MIN_ANGLE = 15;

    // Effective horizontal projectile speed in cm/s as a function of shot distance.
    // Ball speed isn't constant — the LUT increases flywheel speed and flattens the hood
    // as distance grows, so longer shots have higher horizontal velocity.
    //
    // Linear fit from frame-counted time-of-flight measurements:
    //   170 cm -> 255 cm/s   (80 frames @ 120fps)
    //   200 cm -> 312 cm/s   (77 frames @ 120fps)
    // slope = 1.9 cm/s per cm, intercept = -68
    //
    // To re-tune: shoot from a known distance, count frames between ball leaving the hood
    // and impact in slow-mo, compute speed = distance / (frames / fps), refit.
    public static double projectileSpeed(double distanceCm) {
        // Guard: the linear fit goes <= 0 below ~36 cm, which would make atan2 blow up
        // in the lead-angle calculation. Clamp to a sane floor; we never shoot from that close.
        return Math.max(150, 1.9 * distanceCm - 68);
    }

    // Radial-velocity compensation for shoot-on-the-move.
    // Converts robot radial velocity (cm/s, +ve = toward goal) into a flywheel-speed
    // delta (motor ticks/s) to subtract from the static LUT speed.
    //
    // Derived from the 170cm data point: flywheel = 1094 ticks/s produced a ball
    // horizontal speed of 255 cm/s, so 1 cm/s of ball speed costs ~4.3 ticks/s.
    // Re-tune by driving straight at the goal at constant velocity, shooting, and
    // adjusting until shots stop overshooting (too low) or undershooting (too high).
    public static final double FLYWHEEL_TICKS_PER_CM_S = 3;


    public static double getHoodPositionFromDegrees(double degrees) {
        return 0.15 + (degrees - 15) / (75 - 15) * (0.70 - 0.15);
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return velocity * 28 / 60.0;
    }
}
