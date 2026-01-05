package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

public class LightCommand extends Command {
    Telemetry telemetry;
    Lights light;
    boolean lightStatus;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public LightCommand(Lights light, Telemetry telemetry, RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.light = light;
        this.telemetry = telemetry;
        this.pattern = pattern;
    }

    @Override
    public void initialize() {
        lightStatus = false;
    }

    @Override
    public void execute() {
         light.setColor(pattern);
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
