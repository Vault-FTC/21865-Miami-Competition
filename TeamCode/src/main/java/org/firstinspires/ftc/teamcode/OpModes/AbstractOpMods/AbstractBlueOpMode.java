package org.firstinspires.ftc.teamcode.OpModes.AbstractOpMods;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

public abstract class AbstractBlueOpMode extends AbstractOpMode{
    @Override public void setTargets()
    {
        turret.setGoal(new Pose2D(DistanceUnit.CM, 157.5, 152.4, AngleUnit.RADIANS, 0));
        limelight = new LimeLight(hardwareMap, 20);
    }
}
