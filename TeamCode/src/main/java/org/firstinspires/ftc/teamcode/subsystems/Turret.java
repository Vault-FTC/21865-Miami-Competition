package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;

public class Turret extends Subsystem {
    private DcMotorEx turret;
    private final Drivebase drivebase;
    Pose2D goal = Constants.BLUE_CENTER_GOAL;
    double distance;
    private double kP = 0.25;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.1;
    private final double MAX_POWER = 0.6;
    private double power = 0;

    private final ElapsedTime elapsedTime = new ElapsedTime();


    public Turret(HardwareMap hardwareMap, Drivebase drivebase) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorEx.Direction.REVERSE);
        this.drivebase = drivebase;
    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer() {
        elapsedTime.reset();
    }

    public double getAngleError() {
        return drivebase.angleToGoal(drivebase.getPosition(), goal);
    }

    public void update(double error) {
        double angleError = Drivebase.angleToGoal(drivebase.getPosition(), goal);
        double velocityDeg = drivebase.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        drivebase.updateAutoAim(0);
        double offset_by_distance = 0.0;
        distance = drivebase.distanceToGoal(drivebase.getPosition(), goal);
        double errorDeg = (angleError+offset_by_distance) * (180 / Math.PI);
        double new_joystick_rx = errorDeg * kP - velocityDeg * kD;
        drivebase.updateAutoAim(new_joystick_rx);

        double deltaTime = elapsedTime.seconds();
        elapsedTime.reset();

//        if (error == null) {
//            turret.setPower(0);
//            lastError = 0;
//            return;
//        }
        double pTerm = error * kP;
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;


    }

}
