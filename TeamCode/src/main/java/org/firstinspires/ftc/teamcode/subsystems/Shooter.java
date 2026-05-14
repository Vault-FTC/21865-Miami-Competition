package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.LUT;

public class Shooter extends Subsystem {
    public enum CaseModes
    {
        OFF, SHOOT_NEAR, SHOOT_FAR, SHOOT_GATE_CLOSED, REVERSE
    }
    private final DcMotorEx shooter;
    private final ServoGate servoGate;
    private final Servo hood;
    private final Drivebase drivebase;
    private final Intake intake;
    private final LUT lut = new LUT();
    double distance, speed;
    double kP = 0.02;
    double kD = 0.0015;
    CaseModes currentMode = CaseModes.OFF;
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
                double errorDeg = (angleError+offset_by_distance) * (180 / Math.PI);
                double new_joystick_rx = errorDeg * kP - velocityDeg * kD;
                drivebase.updateAutoAim(new_joystick_rx);
                servoGate.openGate();
                if (Math.abs((angleError+offset_by_distance) * ((180/Math.PI))) < 1 && getShooterVelocity() >= distanceToSpeed(distance)) {
                    intake.setState(Intake.CaseModes.ON);
                    gamepad1.rumble(1000);
                }
                break;
            case SHOOT_GATE_CLOSED:
                intake.setState(Intake.CaseModes.OFF);
                servoGate.closeGate();
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
    public double distanceToSpeed(double distanceCm) {
        speed = lut.getSpeed(distanceCm);
        distance = distanceCm;
        return speed;
    }
    public double distanceToHoodPosition(double distanceCm)
    {
        double pos = 0.00423077*distanceCm - 0.443846;
        // clamp to servo limits
        if (pos < 0.1) return 0.1;
        if (pos > 0.7) return 0.7;
        return pos;
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
                + " \n Shooter Speed: " + getShooterVelocity() + " \n " + "Target Speed/Vel: " + distance + ":" + speed;
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
