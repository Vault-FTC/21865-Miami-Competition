package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

public class LightCommand extends Command {
    Telemetry telemetry;
    Lights lights;
    boolean lightStatus;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public LightCommand(Lights light, Telemetry telemetry, RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.lights = light;
        this.telemetry = telemetry;
        this.pattern = pattern;
    }

    @Override
    public void initialize() {
        lightStatus = false;
    }

    @Override
    public void execute() {
         lights.setColor(pattern);
         lightStatus = true;
    }

    @Override
    public boolean isFinished() {
        return lightStatus;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
