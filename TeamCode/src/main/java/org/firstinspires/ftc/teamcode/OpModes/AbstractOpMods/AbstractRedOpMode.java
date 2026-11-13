package org.firstinspires.ftc.teamcode.OpModes.AbstractOpMods;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

public abstract class AbstractRedOpMode extends AbstractOpMode{
    @Override public void setTargets()
    {
        limelight = new Limelight(hardwareMap, 24);
        turret.setGoal( Constants.RED_CENTER_GOAL);
    }
}
