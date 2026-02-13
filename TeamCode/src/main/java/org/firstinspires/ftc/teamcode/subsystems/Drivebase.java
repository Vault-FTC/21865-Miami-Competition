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

    double headingOffsetThingy;

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
// Odometry constants and such
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(205.71207, -15.175, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }


    public void setToCoastMode() {
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void resetHeading(double heading) {
        odo.resetPosAndIMU();
    }
    public Pose2D getPosition() {
        return odo.getPosition();
    }

    public GoBildaPinpointDriver getOdo()
    {
        return odo;
    }


    public String getPositionTelemetry() {
        return "X offset (forwards/backwards): " + odo.getPosX(DistanceUnit.CM) + " Y (left/right): " + odo.getPosY(DistanceUnit.CM) + " Heading: " + odo.getHeading(AngleUnit.DEGREES);
    }

    public void drive(double forward, double right, double rotate, double headingOffset) {
        headingOffsetThingy = headingOffset;
        drive(forward, right, rotate);
    }
    public void drive(double forward, double right, double rotate) {
        double botHeading = -odo.getHeading(AngleUnit.RADIANS) + headingOffsetThingy;
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
        double p = 0.2; //0.1
        double p_rotation = 0.045;
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
        double maxPower = 0.4;

        forwardPower = Math.max(Math.min(forwardPower, maxPower), -maxPower);
        strafePower = Math.max(Math.min(strafePower, maxPower), -maxPower);

        if (Math.abs(forwardPower) < minPower && Math.abs(forwardError) > 3) {
            forwardPower = minPower * Math.signum(forwardError);
        }
        if (Math.abs(strafePower) < minPower && Math.abs(strafeError) > 3) {
            strafePower = minPower * Math.signum(strafeError);
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

}