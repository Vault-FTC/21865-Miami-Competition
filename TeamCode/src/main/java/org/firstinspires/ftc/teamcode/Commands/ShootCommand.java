package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MotorSpeeds;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShootCommand extends Command {
    Telemetry telemetry;
    Shooter shooter;
    Intake intake;
    MotorSpeeds motorSpeed;
    ServoGate servoGate;
    boolean shoot;
    private double startTime;

    public ShootCommand(Shooter shooter, Intake intake, boolean shoot, Telemetry telemetry, MotorSpeeds motorSpeed, ServoGate servoGate) {
        this.shooter = shooter;
        this.intake = intake;
        this.telemetry = telemetry;
        this.motorSpeed = motorSpeed;
        this.shoot = shoot;
        this.servoGate = servoGate;
        addRequirements(this.shooter, this.servoGate);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeedNear(motorSpeed.speed);
        timer.reset();
        startTime = timer.milliseconds();
    }
    @Override
    public void execute() {
        double elapsed = timer.milliseconds() - startTime;
        servoGate.openGate();
        if (elapsed > 2000) {
            intake.spinIntake(0.85);
        } else {
            intake.spinIntake(0);
            shooter.setShooterSpeedNear(motorSpeed.speed);
        }
        telemetry.addData("Running", "Shoot Command");
    }

    public boolean isFinished() {
        return shoot;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterSpeedNear(MotorSpeeds.ZERO.speed);
        intake.spinIntake(0);
        servoGate.closeGate();
  //      intake.spinKicker(0);
    }
}
