package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class Intake extends Subsystem {
    private DcMotorEx intake;
    private DigitalChannel breakbeam1;
    private DigitalChannel breakbeam2;
    CaseModes currentMode = CaseModes.OFF;


    //
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        breakbeam1 = hardwareMap.get(DigitalChannel.class, "breakbeam1");
        breakbeam2 = hardwareMap.get(DigitalChannel.class, "breakbeam2");

        breakbeam1.setMode(DigitalChannel.Mode.INPUT);
        breakbeam2.setMode(DigitalChannel.Mode.INPUT);

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
            case REVERSE:
                intake.setPower(-1);
                break;
        }
    }
    public void setState(CaseModes s) {
        currentMode = s;
    }
    public void spinIntake(double power) {
        intake.setPower(power);
    }

    public double numberOfArtifacts() {
        if (!breakbeam1.getState() && !breakbeam2.getState()) {
            return 2;
        }
        else if (!breakbeam1.getState() && breakbeam2.getState()) {
            return 1;
        } else {
            return 0;
        }
    }
}
