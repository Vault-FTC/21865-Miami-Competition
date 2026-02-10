package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

public class PurePursuiter {
    private Pose2d lastPose = new Pose2d();
    private double lastTimestamp = 0;
    private double followStartTimestamp;
    private Waypoint[][] segments;

    private final PIDController driveController =
            new PIDController(0.05, 0, 0);

    private final PIDController rotController =
            new PIDController(2.0, 0, 0);

    private double lastTargetAngle = 0;

    public enum DriveState {
        IDLE,
        FOLLOWING,
    }

    public DriveState driveState = DriveState.IDLE;

    public int waypointIndex = 0;

    private Path followPath;
    private final ElapsedTime timer = new ElapsedTime();

    private final Supplier< org.firstinspires.ftc.robotcore.external.navigation.Pose2D> currentPoseGetter;
    private final Telemetry t;
    private final GoBildaPinpointDriver odo;

    private Drivebase drivebase;


    public PurePursuiter(Supplier<org.firstinspires.ftc.robotcore.external.navigation.Pose2D> currentPoseSupplier, Drivebase driver, Telemetry t, GoBildaPinpointDriver odometry)
    {
        currentPoseGetter = currentPoseSupplier;
        odo = odometry;
        this.t = t;
        this.drivebase = driver;

    }

    public Pose2d getCurrentPose()
    {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D oldPose = currentPoseGetter.get();
        return new Pose2d(new Vector2d(oldPose.getX(DistanceUnit.CM), oldPose.getY(DistanceUnit.CM)),
                new Rotation2d(oldPose.getHeading(AngleUnit.RADIANS)));
    }

    public void drive(double forward, double strafe, double rotation)
    {
        drivebase.drive(forward, strafe, rotation);
    }

    /**
     * Start following a path
     */
    public void setFollowPath(Path path) {
        waypointIndex = 0;
        driveState = DriveState.IDLE;
        followPath = path;
        segments = followPath.generateLineSegments();
        followStartTimestamp = timer.milliseconds();
    }

    /**
     * Drive to a specific waypoint with PID control
     */
    private void driveToWaypoint(Waypoint targetPoint, boolean useEndpointHeading) {
        Pose2d botPose = getCurrentPose();

        // Calculate vector from robot to target
        Vector2d relativeTargetVector = new Vector2d(
                targetPoint.x - botPose.getY(),
                targetPoint.y - botPose.getX()

        );

        // Use PID to calculate drive speed
        double driveSpeed = driveController.calculate(0, relativeTargetVector.magnitude());

        // Create movement vector and rotate to robot frame
        Vector2d movementSpeed = new Vector2d(
                driveSpeed,
                relativeTargetVector.angle(),
                true
        ).rotate(-botPose.getHeading().getAngleRadians());

        // Determine target heading
        double targetAngle;
        boolean canFlip = false;

        if (useEndpointHeading && targetPoint.targetEndRotation != null &&
                botPose.getPosition().distanceTo(targetPoint)
                        < 10) {
            targetAngle = targetPoint.targetEndRotation.getAngleRadians();
        } else if (targetPoint.targetFollowRotation != null) {
            targetAngle = targetPoint.targetFollowRotation.getAngleRadians();
        } else if (relativeTargetVector.magnitude() > 5) {
            targetAngle = relativeTargetVector.angle() - Math.PI / 2;
            canFlip = true;
        } else {
            targetAngle = lastTargetAngle;
            canFlip = true;
        }
        lastTargetAngle = targetAngle;

        // Calculate rotation error
        double rotError = Rotation2d.getError(targetAngle, botPose.getHeading().getAngleRadians());
        if (rotError > Math.PI && canFlip) {
            rotError = Rotation2d.getError(targetAngle + Math.PI, botPose.getHeading().getAngleRadians());
        }

        // Limit speed
        double magnitude = Range.clip(movementSpeed.magnitude(),
                -targetPoint.maxVelocity,
                targetPoint.maxVelocity);
        movementSpeed = new Vector2d(magnitude, movementSpeed.angle(), false);

        // Calculate rotation speed
        double rotSpeed = rotController.calculate(0, rotError);

        if(t != null)
        {
            t.addData("targetAngle", targetAngle);
            t.addData("RotError", rotError);
            t.addData("TargetVector", relativeTargetVector);
            t.addData("targetPoint", targetPoint.x + " : "  + targetPoint.y);
            t.addData("BotPose", botPose.getX() + " : " + botPose.getY() + " : " + botPose.getHeading());
        }

        // Drive!
        drive(movementSpeed.y, movementSpeed.x, rotSpeed);
    }

    /**
     * Find intersection of lookahead circle with path segment (PURE PURSUIT CORE!)
     */
    private static Waypoint intersection(Pose2d botPose, Waypoint[] lineSegment, double radius) {
        double x1, y1, x2, y2;

        double m = (lineSegment[0].y - lineSegment[1].y) / (lineSegment[0].x - lineSegment[1].x);
        double b = lineSegment[0].y - m * lineSegment[0].x;

        double h = botPose.getX();
        double k = botPose.getY();

        double commonTerm;

        if (Double.isFinite(m)) {
            commonTerm = Math.sqrt(
                    Math.pow(m, 2) * (Math.pow(radius, 2) - Math.pow(h, 2)) +
                            (2 * m * h) * (k - b) +
                            2 * b * k +
                            Math.pow(radius, 2) -
                            Math.pow(b, 2) -
                            Math.pow(k, 2)
            );

            x1 = (m * (k - b) + h + commonTerm) / (Math.pow(m, 2) + 1);
            x2 = (m * (k - b) + h - commonTerm) / (Math.pow(m, 2) + 1);

            y1 = m * x1 + b;
            y2 = m * x2 + b;
        } else {
            x1 = lineSegment[0].x;
            commonTerm = Math.sqrt(Math.pow(radius, 2) - Math.pow((x1 - h), 2));
            y1 = botPose.getY() + commonTerm;
            x2 = x1;
            y2 = botPose.getY() - commonTerm;
        }

        Waypoint point0 = new Waypoint(x1, y1, 0,
                lineSegment[1].targetFollowRotation,
                lineSegment[1].targetEndRotation,
                lineSegment[1].maxVelocity);
        Waypoint point1 = new Waypoint(x2, y2, 0,
                lineSegment[1].targetFollowRotation,
                lineSegment[1].targetEndRotation,
                lineSegment[1].maxVelocity);

        Pair<Waypoint, Double> intersection0 = new Pair<>(
                point0, getTValue(lineSegment[0], lineSegment[1], point0));
        Pair<Waypoint, Double> intersection1 = new Pair<>(
                point1, getTValue(lineSegment[0], lineSegment[1], point1));

        Pair<Waypoint, Double> bestIntersection =
                intersection0.second > intersection1.second ? intersection0 : intersection1;

        if (bestIntersection.second > 1) {
            return null;
        }

        return bestIntersection.first;
    }

    private static double getTValue(Vector2d point1, Vector2d point2, Vector2d interpolationPoint) {
        if (Math.abs(point1.x - point2.x) < 0.001) {
            return (interpolationPoint.y - point1.y) / (point2.y - point1.y);
        }
        return (interpolationPoint.x - point1.x) / (point2.x - point1.x);
    }

    /**
     * Main path following method - call this in your loop!
     */
    public void followPath() {
        odo.update();
        Pose2d botPose = getCurrentPose();
        Waypoint targetPoint;
        boolean endOfPath = false;

        switch (driveState) {
            case IDLE:
                if (waypointIndex != 0) return;
                followStartTimestamp = timer.milliseconds();
                driveState = DriveState.FOLLOWING;

            case FOLLOWING:
                if (timer.milliseconds() > followStartTimestamp + followPath.timeout) {
                    driveState = DriveState.IDLE;
                    return;
                }

                targetPoint = intersection(botPose, segments[waypointIndex],
                        segments[waypointIndex][1].followRadius);

                if (targetPoint == null) {
                    if (waypointIndex == segments.length - 1) {
                        targetPoint = segments[segments.length - 1][1];
                        endOfPath = true;
                    } else {
                        waypointIndex++;
                        followPath();
                        return;
                    }
                }
                driveToWaypoint(targetPoint, endOfPath);
                break;
        }
    }

    /**
     * Check if path following is complete
     */
    public boolean finishedFollowing() {
        if (timer.milliseconds() > followStartTimestamp + followPath.timeout &&
                timer.milliseconds() > followStartTimestamp + 1) {
            return true;
        }

        Pose2d botPose = getCurrentPose();
        double currentTimestamp = timer.milliseconds();

        double speed = botPose.getPosition().distanceTo(lastPose.getPosition())
                /
                (currentTimestamp - lastTimestamp) * 1000;

        lastTimestamp = currentTimestamp;

        boolean atEndpoint = speed < 2.0 &&
                botPose.getPosition().distanceTo(segments[segments.length - 1][1]) < 2.0 &&
                waypointIndex == segments.length - 1;

        boolean atTargetHeading;
        if (segments[segments.length - 1][1].targetEndRotation == null) {
            atTargetHeading = true;
        } else {
            atTargetHeading = Math.abs(Rotation2d.getError(
                    segments[segments.length - 1][1].targetEndRotation.getAngleRadians(),
                    botPose.getHeading().getAngleRadians()
            )) < Math.toRadians(5);
        }

        lastPose = botPose;
        return atEndpoint && atTargetHeading;
    }

    /**
     * Get remaining distance to end of path
     */
    public double remainingDistance() {
        if (driveState == DriveState.IDLE) {
            return 0;
        }
        Pose2d currentPose = getCurrentPose();
        double distance = currentPose.getPosition().distanceTo(segments[waypointIndex][1]);
        for (int i = waypointIndex + 1; i < segments.length; i++) {
            distance += segments[i][0].distanceTo(segments[i][1]);
        }
        return distance;
    }

    private static class Pair<T, U> {
        public final T first;
        public final U second;

        public Pair(T first, U second) {
            this.first = first;
            this.second = second;
        }
    }
}
