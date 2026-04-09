package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class Turret extends Subsystem {
    private DcMotorEx turret;
    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.1;
    private final double MAX_POWER = 0.6;
    private double power = 0;

    private final ElapsedTime elapsedTime = new ElapsedTime();

    public Turret(HardwareMap map)
    {
        init(map);
    }

    private void init(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newKD) {
        kP = newKD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer() {
        elapsedTime.reset();
    }

    public void update(double error) {
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
