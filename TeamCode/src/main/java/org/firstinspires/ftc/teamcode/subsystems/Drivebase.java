package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.PurePursuit.PIDController;
import org.firstinspires.ftc.teamcode.PurePursuit.Path;
import org.firstinspires.ftc.teamcode.PurePursuit.Rotation2d;
import org.firstinspires.ftc.teamcode.PurePursuit.Vector2d;
import org.firstinspires.ftc.teamcode.PurePursuit.Waypoint;
import org.firstinspires.ftc.teamcode.PurePursuit.Pose2d;


public class Drivebase extends Subsystem {
    private final DcMotorEx frmotor, flmotor, brmotor, blmotor;
    GoBildaPinpointDriver odo;
    public int waypointIndex = 0;
    private Path followPath;
    private final ElapsedTime timer = new ElapsedTime();
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

    public Drivebase(HardwareMap hardwareMap) {
        frmotor = hardwareMap.get(DcMotorEx.class, "rf");
        flmotor = hardwareMap.get(DcMotorEx.class, "lf");
        brmotor = hardwareMap.get(DcMotorEx.class, "rb");
        blmotor = hardwareMap.get(DcMotorEx.class, "lb");

        frmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// Reverse one side of motors if needed (depends on robot configuration)
        frmotor.setDirection(DcMotorEx.Direction.REVERSE);
        brmotor.setDirection(DcMotorEx.Direction.REVERSE);
        flmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

         /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.

        robot is 350.35000mm wide
        365.71207mm long
         */
        odo.setOffsets(205.71207, -15.175, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    public void resetHeading(double heading) {
        odo.resetPosAndIMU();
    }
    public Pose2D getPosition() {
        return odo.getPosition();
    }


    public String getPositionTelemetry() {
        return "X offset (forwards/backwards): " + odo.getPosX(DistanceUnit.CM) + " Y (left/right): " + odo.getPosY(DistanceUnit.CM) + " Heading: " + odo.getHeading(AngleUnit.DEGREES);
    }

    public void drive(double forward, double right, double rotate) {
        double botHeading = -odo.getHeading(AngleUnit.RADIANS);
//         X is positive up, Y is positive to the right
        double rotRight = right * Math.cos(botHeading) - forward * Math.sin(botHeading);
        double rotForward = right * Math.sin(botHeading) + forward * Math.cos(botHeading);

        double frontLeftPower = rotForward + rotRight + rotate;
        double backLeftPower = rotForward - rotRight + rotate;
        double frontRightPower = rotForward - rotRight - rotate;
        double backRightPower = rotForward + rotRight - rotate;

        // Calculate motor powers

        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftPower = Math.abs(frontLeftPower) < 0.02 ? 0 : frontLeftPower;
        frontRightPower = Math.abs(frontRightPower) < 0.02 ? 0 : frontRightPower;
        backLeftPower = Math.abs(backLeftPower) < 0.02 ? 0 : backLeftPower;
        backRightPower = Math.abs(backRightPower) < 0.02 ? 0 : backRightPower;

        // Set motor powers
        frmotor.setPower(frontRightPower);
        brmotor.setPower(backRightPower);
        flmotor.setPower(frontLeftPower);
        blmotor.setPower(backLeftPower);
    }

    public void driveToPosition(Location target, double turnVal, Telemetry telemetry) {
        double p = 0.006;
        double p_rotation = 0.03;
        double strafe = (target.Strafe - odo.getPosY(DistanceUnit.CM));
        double forward = (-target.Forward + odo.getPosX(DistanceUnit.CM));
        double heading = (target.TurnDegrees - odo.getHeading(AngleUnit.DEGREES));

        double strafeError = (target.Strafe - odo.getPosY(DistanceUnit.CM));
        double forwardError = (-target.Forward + odo.getPosX(DistanceUnit.CM));
        double headingError = (target.TurnDegrees - odo.getHeading(AngleUnit.DEGREES));


        if (telemetry != null) {
            telemetry.addData("Target", "X: " + target.Strafe + "  Y: " + target.Forward);
        }

        double forwardPower = forward * p;
        double strafePower = strafe * p;
        double turnPower = heading * p_rotation;

        double minPower = 0.25;
        double maxPower = 1;

        forwardPower = Math.max(Math.min(forwardPower, maxPower), -maxPower);
        strafePower = Math.max(Math.min(strafePower, maxPower), -maxPower);

        if (Math.abs(forwardPower) < minPower && Math.abs(forwardError) > 3) {
            forwardPower = minPower * Math.signum(forwardError);
        }
        if (Math.abs(strafePower) < minPower && Math.abs(strafeError) > 3) {
            strafePower = minPower * Math.signum(strafeError);
        }

        double distance = Math.hypot(forward,strafe);

        if (distance < 7) {
            drive(0,0,0);
            return;
        }

        drive(forwardPower, strafePower, turnPower);

    }

    public boolean isAtPosition(Location target) {
        return isAtPosition(target, 15, 17.5);
    }

    public boolean isAtPosition(Location target, double toleranceXY, double toleranceAngle) {
        double currentX = odo.getPosX(DistanceUnit.CM);
        double currentY = odo.getPosY(DistanceUnit.CM);
        double currentHeading = odo.getHeading(AngleUnit.DEGREES);
        return Math.abs(currentY - target.Strafe) < toleranceXY &&
                Math.abs(currentX - target.Forward) < toleranceXY &&
                Math.abs(currentHeading-target.TurnDegrees) < toleranceAngle;
    }

    private Pose2d getCurrentPose() {
        return new Pose2d(
                new Vector2d(
                        odo.getPosX(DistanceUnit.CM),
                        odo.getPosY(DistanceUnit.CM)
                ),
                new Rotation2d(odo.getHeading(AngleUnit.DEGREES))
        );
    }

    public void setCurrentPose(Pose2D pos)
    {
        odo.setPosition(pos);
    }

    public void setCurrentPose(double x, double y, double radians)
    {
        setCurrentPose(new Pose2D(DistanceUnit.CM, x, y, AngleUnit.RADIANS, radians));
    }

    public void setCurrentPose(double x, double y)
    {
        odo.setPosX(x, DistanceUnit.CM);
        odo.setPosY(y, DistanceUnit.CM);
    }

    public void update() {
        odo.update(); // updates the odometry internally
    }

    public LLResult update(LimeLight limeLight) {
        odo.update();
        limeLight.limelight.updateRobotOrientation(odo.getHeading(AngleUnit.DEGREES));
        LLResult april = limeLight.getResult();
        if (april == null) {
            return null;
        }
        Pose3D pose = april.getBotpose_MT2();
//        if (pose == null) {
//            return null;
//        }
        //setCurrentPose(pose.getPosition().x, pose.getPosition().y, pose.getOrientation().getYaw(AngleUnit.RADIANS));
        return april;
    }

    public static double distanceToGoal(Pose2D robot, Pose2D goal) {
        double dx = robot.getX(DistanceUnit.CM) - goal.getX(DistanceUnit.CM);
        double dy = robot.getY(DistanceUnit.CM) - goal.getY(DistanceUnit.CM);
        return Math.hypot(dx, dy);
    }
    public static double angleToGoal(Pose2D robot, Pose2D goal) {
        double dx = goal.getX(DistanceUnit.CM) - robot.getX(DistanceUnit.CM);
        double dy = goal.getY(DistanceUnit.CM) - robot.getY(DistanceUnit.CM);
        double goalHeading = Math.atan2(dy, dx); // radians
        double robotHeading = robot.getHeading(AngleUnit.RADIANS);
        double error = goalHeading - robotHeading;

        // Normalize to [-π, π]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        return error;
    }


    // PURE PURSUIT STARTS HERE


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
                targetPoint.x - botPose.getX(),
                targetPoint.y - botPose.getY()

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