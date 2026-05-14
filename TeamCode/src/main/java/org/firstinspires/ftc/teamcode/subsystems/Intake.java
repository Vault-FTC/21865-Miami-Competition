package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class Intake extends Subsystem {
    public enum CaseModes
    {
        OFF, REVERSE, ON, HALF_SPEED, SIXTY_PERCENT_SPEED
    }
    private DcMotorEx intake;
    CaseModes currentMode = CaseModes.OFF;


    //
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void update()
    {
        switch(currentMode) {
            case ON:
                intake.setPower(1);
                break;
            case OFF:
                intake.setPower(0);
                break;
            case HALF_SPEED:
                intake.setPower(0.5);
                break;
            case SIXTY_PERCENT_SPEED:
                intake.setPower(0.6);
                break;
            case REVERSE:
                intake.setPower(-1);
                break;
        }
    }

    public void setState(CaseModes s) {
        currentMode = s;
    }

    public boolean hasThreeArtifacts() {
        return true; //placeholder
    }
    public double currentDraw() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
}
