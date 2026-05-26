package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.LUT;

public class Shooter extends Subsystem {
    public enum CaseModes
    {
        OFF, SHOOT_NEAR, SHOOT_FAR, SHOOT_GATE_CLOSED, REVERSE, SHOOT_ON_MOVE, SHOOT_NO_AIM, SHOOT_LIMELIGHT_AIM
    }
    private final DcMotorEx shooter;
    private final ServoGate servoGate;
    private final Servo hood;
    private final Drivebase drivebase;
    private final Intake intake;
    private final LUT lut = new LUT();
    private LimeLight limelight = null;
    double distance, speed;
    double kP = 1.3;
    double kD = 0.0015;
    CaseModes currentMode = CaseModes.OFF;

    /** Manual aim trim set by the operator via gamepad2.  Positive = aim further right. */
    private double aimTrimDeg = 0.0;
    Pose2D goal = Constants.BLUE_CENTER_GOAL;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(250, 0, 0, 15);
    Gamepad gamepad1;

    public Shooter(HardwareMap hardwareMap, Drivebase driveBase, ServoGate servoGate, Intake intake, Gamepad gamepad) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        hood = hardwareMap.get(Servo.class, "hood");
        this.servoGate = servoGate;
        this.drivebase = driveBase;
        this.intake = intake;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        gamepad1 = gamepad;
    }

    public void setGoal(Pose2D goal) {
        this.goal = goal;
    }

    public void update() {
        double angleError = Drivebase.angleToGoal(drivebase.getPosition(), goal);
        double velocityDeg = drivebase.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        drivebase.updateAutoAim(0);
        double offset_by_distance = 0.0;
        distance = drivebase.distanceToGoal(drivebase.getPosition(), goal);
        setHoodPosition(distanceToHoodPosition(distance));
        setShooterSpeedNear(distanceToSpeed(distance));
        switch(currentMode){
            case OFF:
                shooter.setVelocity(0);
                servoGate.closeGate();
                break;
            case SHOOT_FAR:
                offset_by_distance = 0.05;
            case SHOOT_NEAR:
                double correctedError = angleError + offset_by_distance + Math.toRadians(aimTrimDeg);
                double new_joystick_rx = correctedError * kP - velocityDeg * kD;
                drivebase.updateAutoAim(new_joystick_rx);
                servoGate.openGate();
                if (Math.abs(Math.toDegrees(correctedError)) < 1 && getShooterVelocity() >= distanceToSpeed(distance)) {
                    intake.setState(Intake.CaseModes.ON);
                    gamepad1.rumble(1000);
                }
                break;
            case SHOOT_ON_MOVE: {
                // Compute lead angle from robot translational velocity in the field frame.
                double velX = drivebase.getOdo().getVelX(DistanceUnit.CM);
                double velY = drivebase.getOdo().getVelY(DistanceUnit.CM);
                double robotX = drivebase.getPosition().getX(DistanceUnit.CM);
                double robotY = drivebase.getPosition().getY(DistanceUnit.CM);
                double goalAngle = Math.atan2(
                        goal.getY(DistanceUnit.CM) - robotY,
                        goal.getX(DistanceUnit.CM) - robotX);
                // Signed velocity component perpendicular to the robot->goal vector.
                // Positive means the robot is drifting "left" of the aim line; we lead the same way.
                double vPerp = -velX * Math.sin(goalAngle) + velY * Math.cos(goalAngle);
                // TEMP: radial compensation disabled for tuning lead angle in isolation.
                // Re-enable by uncommenting the vRadial / targetFlywheel block below and
                // replacing distanceToSpeed(distance) in the readiness check with targetFlywheel.
                // double vRadial = velX * Math.cos(goalAngle) + velY * Math.sin(goalAngle);
                double leadAngle = Math.atan2(-vPerp, ShooterConstants.projectileSpeed(distance));
                double sotmError = angleError + leadAngle + Math.toRadians(aimTrimDeg);
                double sotm_joystick_rx = sotmError * kP - velocityDeg * kD;
                drivebase.updateAutoAim(sotm_joystick_rx);
                // double targetFlywheel = distanceToSpeed(distance)
                //         - vRadial * ShooterConstants.FLYWHEEL_TICKS_PER_CM_S;
                // setShooterSpeedNear(targetFlywheel);
                servoGate.openGate();
                // Looser angle tolerance than SHOOT_NEAR since we're firing while moving.
                if (Math.abs(Math.toDegrees(sotmError)) < 2.5
                        && getShooterVelocity() >= distanceToSpeed(distance)) {
                    intake.setState(Intake.CaseModes.SIXTY_PERCENT_SPEED);
                    gamepad1.rumble(1000);
                }
                break;
            }
            case SHOOT_GATE_CLOSED:
                intake.setState(Intake.CaseModes.OFF);
                servoGate.closeGate();
                break;
            case SHOOT_NO_AIM:
                servoGate.openGate();
                if (getShooterVelocity() >= 1100) {
                    intake.setState(Intake.CaseModes.ON);
                }
                break;
            case SHOOT_LIMELIGHT_AIM:
                // Backup aim using Limelight tx (horizontal offset to target) instead of odometry.
                // tx > 0 means target is right of crosshair → rotate right to center it.
                servoGate.openGate();
                if (limelight != null) {
                    double tx = limelight.getTx() + aimTrimDeg; // trim is also in degrees
                    double txRad = Math.toRadians(tx);
                    double llRx = txRad * kP - velocityDeg * kD;
                    drivebase.updateAutoAim(llRx);
                    // Fire once the target is centred (±2°) and the flywheel is up to speed.
                    if (Math.abs(tx) < 2.0 && getShooterVelocity() >= distanceToSpeed(distance)) {
                        intake.setState(Intake.CaseModes.ON);
                        gamepad1.rumble(1000);
                    }
                }
                break;
            case REVERSE:
                servoGate.openGate();
                intake.setState(Intake.CaseModes.REVERSE);
                shooter.setVelocity(-900);
                break;
        }
    }

    public void setState(CaseModes s) {
        currentMode = s;
    }

    /** Call this after both Shooter and LimeLight are constructed (e.g. in startHardware). */
    public void setLimelight(LimeLight ll) {
        limelight = ll;
    }

    /**
     * Nudges the aim trim by {@code deltaDeg} degrees.
     * Positive = aim further right, negative = aim further left.
     * Call each loop while the operator holds the trim button.
     */
    public void adjustAimTrim(double deltaDeg) {
        aimTrimDeg += deltaDeg;
    }

    /** Resets the aim trim to zero (operator pressed the reset button). */
    public void resetAimTrim() {
        aimTrimDeg = 0.0;
    }

    /** Returns the current aim trim in degrees (for telemetry). */
    public double getAimTrimDeg() {
        return aimTrimDeg;
    }
    public double distanceToSpeed(double distanceCm) {
        speed = lut.getSpeed(distanceCm);
        distance = distanceCm;
        return speed;
    }
    public double distanceToHoodPosition(double distanceCm)
    {
        if (distanceCm <= 100) {
            return 0.15;
        } else if (distanceCm <= 275) {
            return 0.5;
        } else {
            return 0.7;
        }
//        double pos = 0.003*distanceCm - 0.1; //double pos = 0.00423077*distanceCm - 0.443846;
//        // clamp to servo limits
//        if (pos < 0.1) return 0.1;
//        if (pos > 0.7) return 0.7;
//        return pos;
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
        hood.setPosition(position);
    }
    public String telemetryUpdate() {
        return "Servo Position: " + hood.getPosition() + " \n ShooterMode: " + currentMode
                + " \n Shooter Speed: " + getShooterVelocity() + " \n " + "Target Speed/Vel: " + distance + ":" + speed
                + String.format(" \n Aim trim: %.1f°  (g2 LB/RB to adjust, Start to reset)", aimTrimDeg);
    }
    public double getShooterVelocity() {
        return shooter.getVelocity();
    }

    double speedToTicks(double velocity) {
        double rpm = 1.995 * velocity - 611.76;
        double ticksPerSecond = (rpm * 28)/60;
        return ticksPerSecond;
    }
    double degreesToPosition(double degrees) {
        return (0.1-0.7)/(0.63-0.42) * degrees + 66.5;
    }
    double newDistanceToSpeed(double distance) {
        double g = 981.0; // cm/s^2
        double x = distance;
        double y = ShooterConstants.SCORE_HEIGHT;
        double phi = ShooterConstants.SCORE_ANGLE;
        double theta = Math.atan((2*y/x) - Math.tan(phi));

        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y);
        if (denominator <=0 ) {
            return 800;
        } else {
            double v = Math.sqrt((g*x*x) / denominator);
            return speedToTicks(v);
        }
    }

    double calculatedVelocity(double distance) {
        double g = 981.0; // cm/s^2
        double x = distance;
        double y = ShooterConstants.SCORE_HEIGHT;
        double phi = ShooterConstants.SCORE_ANGLE;
        double theta = Math.atan(2*y/x) - Math.tan(phi);

        double denominator = Math.pow(Math.cos(theta), 2) * (Math.tan(theta) - Math.tan(phi));
        if (denominator <=0 ) {
            return -1;
        }
        double v = Math.sqrt((g*x) / denominator);
        return v;
    }
}
