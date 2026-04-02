package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.CaseModes;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class TimedShootCommand extends Command {
    Telemetry telemetry;
    Shooter shooter;
    Intake intake;
    private final ServoGate servoGate;

    double hoodPosition;
    double motorSpeed;
    private final double durationMs;
    private double startTime;
    CaseModes intakeSetting;
    double currentTime = 0;
    public TimedShootCommand(Shooter shooter, Intake intake, double durationSeconds, Telemetry telemetry, double motorSpeed, ServoGate servoGate, double intakeSpeed, double hoodPosition) {
        this.shooter = shooter;
        this.hoodPosition = hoodPosition;
        this.intake = intake;
        this.servoGate = servoGate;
        this.telemetry = telemetry;
        this.motorSpeed = motorSpeed;
        this.durationMs = durationSeconds * 1000;
        if(intakeSpeed > 0.8)
        {
            intakeSetting = CaseModes.ON;
        }
        else
        {
            intakeSetting = CaseModes.SIXTY_PERCENT_SPEED;
        }
        addRequirements(this.shooter, this.intake);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeedNear(motorSpeed);
        timer.reset();
        startTime = timer.milliseconds();
    }
    @Override
    public void execute() {
        shooter.setHoodPosition(hoodPosition);
        currentTime = timer.milliseconds();
        double elapsed = currentTime - startTime;

        servoGate.openGate();
        if (shooter.getShooterVelocity() >= motorSpeed) {
            intake.setState(intakeSetting);
        } else {
            shooter.setShooterSpeedNear(motorSpeed);
        }
        telemetry.addData("Running", "Shoot Command");
    }

    public boolean isFinished() {
        return timer.milliseconds() - startTime >= durationMs;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(CaseModes.OFF);
    //    intake.spinKicker(0);
    }

}
