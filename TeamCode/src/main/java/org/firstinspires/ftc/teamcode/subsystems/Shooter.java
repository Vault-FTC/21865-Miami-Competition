package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class Shooter extends Subsystem {
 //   boolean spinKicker = true;
    MotorSpeeds currentSpeed = MotorSpeeds.NEAR;
//    private DcMotorEx kicker;
    private DcMotorEx shooter;
    private Servo hoodServo;
    double lastTime = 0;
    double lastTargetVelocity = 0;
    double kA = 0.5; // tune this experimentally
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(20.0004, 0, 0, 18.2);
    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    public double distanceToSpeed(double distanceCm)
    {
        return  ((distanceCm * 100.0 / 2.54) - 40) * 5 + 700;
        //return (585 * Math.pow(1.0046834253, (distanceCm / 2.54)));
    }
    public void setPower(double speed) {
        shooter.setPower(speed);
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

        // Approximate acceleration
        double acceleration = (targetVelocity - lastTargetVelocity) / dt;

        // Base PIDF power (FTC PIDF takes care of P/I/D/F automatically)
        double pidfPower = targetVelocity;

        // Add acceleration feedforward term
        double extraPower = kA * acceleration;

        // Combine
        double finalVelocity = pidfPower + extraPower;

        shooter.setVelocity(finalVelocity);

        lastTargetVelocity = targetVelocity;
        lastTime = currentTime;
    }
    public double getShooterVelocity()
    {
        return shooter.getVelocity();
    }
}

