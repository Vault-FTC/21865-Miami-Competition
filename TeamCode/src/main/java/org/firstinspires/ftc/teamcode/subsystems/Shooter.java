package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.LUT;

import java.util.Vector;

public class Shooter extends Subsystem {
    private static final double CLOSE_DIST_CM = 140;
    private static final double MID_DIST_CM = 270;
    private static final double FAR_DIST_CM = 370;
    private static final double CLOSE_SPEED = 1100;
    private static final double MID_SPEED   = 1300;
    private static final double FAR_SPEED   = 2000;
    private static final double CLOSE_HOOD = 0.15;
    private static final double MID_HOOD = 0.5; //0.2
    private static final double FAR_HOOD = 0.7; //0.5
    private final DcMotorEx shooter;
    private final Servo hoodServo;

    private final LUT lut = new LUT();
    double lastTime = 0;
    double lastTargetVelocity = 0;
    double kA = 0.5; // tune this experimentally
    double distance, speed;

    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(250, 0, 0, 15);
    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    public double distanceToSpeed(double distanceCm) {
        speed = lut.getSpeed(distanceCm);
        distance = distanceCm;
        return speed;
    }
    public double distanceToHoodPosition(double distanceCm)
    {
        if (distanceCm <= CLOSE_DIST_CM) {
            return CLOSE_HOOD;
        } else if (distanceCm <= MID_DIST_CM) {
            return MID_HOOD;
        } else {
            return FAR_HOOD;
        }
    }
    public void setShooterSpeedNear(double speed){
        pidfCoefficients = new PIDFCoefficients(250, 0, 0, 15);
        shooter.setVelocity(speed);
    }
    public void setShooterSpeedFar(double speed) {
        pidfCoefficients = new PIDFCoefficients(350, 0, 0, 15.2); // p:440 f:14  for new shooter if we need to change it
        shooter.setVelocity(speed);
    }
    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }
    public String telemetryUpdate() {
        return "Servo Position: " + hoodServo.getPosition()
                + " \n Shooter Speed: " + getShooterVelocity() + " \n " + "Target Speed/Vel: " + distance + ":" + speed;
    }
    public double getShooterVelocity()
    {
        return shooter.getVelocity();
    }

    double degreesToPosition(double degrees) {
        return 0.15 + (degrees - 15) / (75 - 15) * (0.70 - 0.15);
    }

    public double updateShootOnMove(GoBildaPinpointDriver pinpoint, double GOAL_X, double GOAL_Y) {
        Pose2D pose = pinpoint.getPosition();
        double velocityX = pinpoint.getVelX(DistanceUnit.CM);
        double velocityY = pinpoint.getVelY(DistanceUnit.CM);
        double robotVelocityMag = Math.sqrt((velocityX*velocityX) + (velocityY*velocityY));
        double robotToGoalX = pinpoint.getPosX(DistanceUnit.CM) - GOAL_X;
        double robotToGoalY = pinpoint.getPosY(DistanceUnit.CM) - GOAL_Y;
        double rawDistance = Math.sqrt(robotToGoalX * robotToGoalX + robotToGoalY * robotToGoalY);

        // constants
        double g = 981;
        double x = rawDistance - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;

        // initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), ShooterConstants.HOOD_MAX_ANGLE, ShooterConstants.HOOD_MIN_ANGLE);
        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));


        double coordinateTheta = Math.atan2(velocityY, velocityX) - Math.atan2(robotToGoalY, robotToGoalX);

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocityMag;
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocityMag;

        double velocityZ = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(Math.atan(velocityZ / nvr), ShooterConstants.HOOD_MAX_ANGLE, ShooterConstants.HOOD_MIN_ANGLE);
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
        setHoodPosition(ShooterConstants.getHoodPositionFromDegrees(hoodAngle));
        setShooterSpeedNear(flywheelSpeed);

        double robotVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double goalTheta = Math.atan2(robotToGoalY, robotToGoalX);
        double robotAngle = Math.toDegrees(goalTheta - robotVelCompOffset - pinpoint.getHeading(AngleUnit.RADIANS));
        return robotAngle;
    }
}

