package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

public abstract class AbstractBlueOpMode extends AbstractOpMode{
    @Override public void setTargets()
    {
        turret.setGoal(Constants.BLUE_CENTER_GOAL);
        limelight = new LimeLight(hardwareMap, 20);
    }
}
