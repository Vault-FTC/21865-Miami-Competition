package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

public abstract class AbstractRedOpMode extends AbstractOpMode{
    @Override public void setTargets()
    {
        limelight = new LimeLight(hardwareMap, 24);
        turret.setGoal( Constants.RED_CENTER_GOAL);
    }
}
