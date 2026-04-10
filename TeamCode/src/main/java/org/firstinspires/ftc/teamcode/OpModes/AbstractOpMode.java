package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public abstract class AbstractOpMode extends LinearOpMode {

    public Intake intake;
    public Lights lights;
    public Drivebase drivebase;
    public ServoGate servoGate;
    public Shooter shooter;
    public LimeLight limelight;
    public Turret turret;
    public Turret Stalin;

    public void startHardware()
    {
        intake = new Intake(hardwareMap);
        lights = new Lights(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        servoGate = new ServoGate(hardwareMap);
        shooter = new Shooter(hardwareMap, drivebase, servoGate, intake, gamepad1);
        Stalin = new Turret(hardwareMap);
    }

}
