package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.driveallclass;

public class DriveToCommand extends Command {
    Telemetry telemetry;
    private final driveallclass drive;
    private final Location target;

    public DriveToCommand(driveallclass drive, Location target, Telemetry telemetry) {
        this.drive = drive;
        this.target = target;
        this.telemetry = telemetry;
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        drive.driveToPosition(target, target.TurnDegrees, telemetry);
        telemetry.addData("Running", "Drive Command");
    }

    public boolean isFinished() {
        return drive.isAtPosition(target);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0,0,0);
    }
}
