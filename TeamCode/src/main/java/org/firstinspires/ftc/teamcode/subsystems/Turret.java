package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;

public class Turret extends Subsystem {
    private DcMotorEx turret;
    private final Drivebase drivebase;
    double distance;
    private double kP = 0.25;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.1;
    private final double MAX_POWER = 0.6;
    private double power = 0;
    Pose2D goal = Constants.BLUE_CENTER_GOAL;

    private final double TURRET_TICKS_TO_RADIANS = Math.PI * 2.0 / 580.0;

    private final ElapsedTime elapsedTime = new ElapsedTime();

    Telemetry telemetry;

    public Turret(HardwareMap hardwareMap, Drivebase drivebase, Telemetry telemetry) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorEx.Direction.REVERSE);
        this.drivebase = drivebase;
        this.telemetry = telemetry;
    }

    public void setGoal(Pose2D input)
    {
        goal = input;
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

    public double getTurretAngle() {
        double targetAngle = Math.atan2(goal.getY(DistanceUnit.CM) - drivebase.getPosition().getY(DistanceUnit.CM), goal.getX(DistanceUnit.CM) - drivebase.getPosition().getX(DistanceUnit.CM));
        double turretAngle = AngleUnit.normalizeRadians(targetAngle - drivebase.getPosition().getHeading(AngleUnit.RADIANS));
        telemetry.addData("TurretAngle", turretAngle);
        telemetry.addData("TargetAngle", targetAngle);
        return turretAngle;
    }

    public double angleToTicks(double angle) {
        return angle * 0.1008; // Angle * ticks/degree of big turret ring
    }

    public void update(double error) {
        double angleError = Drivebase.angleToGoal(drivebase.getPosition(), goal);

        double turretPosTicks = turret.getCurrentPosition();
        double turretPosRadians = turretPosTicks * TURRET_TICKS_TO_RADIANS;
        error = AngleUnit.normalizeRadians(getTurretAngle() + turretPosRadians);

//        double velocityDeg = drivebase.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
//        drivebase.updateAutoAim(0);
//        double offset_by_distance = 0.0;
//        distance = drivebase.distanceToGoal(drivebase.getPosition(), goal);
//        double errorDeg = (angleError+offset_by_distance) * (180 / Math.PI);
//        double new_joystick_rx = errorDeg * kP - velocityDeg * kD;
//        drivebase.updateAutoAim(new_joystick_rx);

        double deltaTime = elapsedTime.seconds();
        elapsedTime.reset();

        double pTerm = error * kP;
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = -Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;
        telemetry.addData("Error", error);
        telemetry.addData("TurretDegrees", Math.toDegrees(turretPosRadians));
    }

}
