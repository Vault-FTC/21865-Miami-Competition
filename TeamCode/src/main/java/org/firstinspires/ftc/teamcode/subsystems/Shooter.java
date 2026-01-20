package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.FieldConstants;

public class Shooter extends Subsystem {
    private static final double CLOSE_DIST_CM = 140;
    private static final double MID_DIST_CM   = 240;

    private static final double CLOSE_SPEED = 1100;
    private static final double MID_SPEED   = 1300;
    private static final double FAR_SPEED   = 1750;
    private static final double CLOSE_HOOD = 0.1;
    private static final double MID_HOOD = 0.25; //0.2
    private static final double FAR_HOOD = 0.1; //0.5
    private final DcMotorEx shooter;
    private final Servo hoodServo;
    double lastTime = 0;
    double lastTargetVelocity = 0;
    double kA = 0.5; // tune this experimentally
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(250, 0, 0, 15);
    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    public double distanceToSpeed(double distanceCm)
    {
        if (distanceCm <= CLOSE_DIST_CM) {
            return CLOSE_SPEED;
        } else if (distanceCm <= MID_DIST_CM) {
            return MID_SPEED;
        } else {
            return FAR_SPEED;
        }
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

    public void setShooterSpeed(double speed){
        shooter.setVelocity(speed);
    }

/** wait until we have hood assembled to implement autoSetHoodAngle**/
//    public void autoSetHoodAngle (Limelight3A limelight3A, int id) {
//        LLResult result = limelight3A.getLatestResult();
//        double distanceToTarget = result.getT
//        double targetAngle = odo.;
//        double StrafeDistance_3D = id.getRobotPoseTargetSpace().getY();
//    }
    public void setShooterVelocityDynamic(double targetVelocity) {
        double currentVelocity = shooter.getVelocity();
        double currentTime = System.nanoTime() / 1e9; // seconds
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.001;

        double acceleration = (targetVelocity - lastTargetVelocity) / dt;

        double pidfPower = targetVelocity;

        double extraPower = kA * acceleration;

        double finalVelocity = pidfPower + extraPower;

        shooter.setVelocity(finalVelocity);

        lastTargetVelocity = targetVelocity;
        lastTime = currentTime;
    }
    public void raiseHood(){
        hoodServo.setPosition(0);
    }
    public void lowerHood() {
        hoodServo.setPosition(1);
    }
    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }
    public String telemetryUpdate() {
        return "Servo Position: " + hoodServo.getPosition() + "Shooter Speed: " + getShooterVelocity();
    }
    public void stop(){
        hoodServo.setPosition(0.5);
        }

    public double getShooterVelocity()
    {
        return shooter.getVelocity();
    }
}

