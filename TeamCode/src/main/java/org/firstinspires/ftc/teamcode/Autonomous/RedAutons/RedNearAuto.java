package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearAuto;

@Autonomous(name = "Red Near Auto")
public class RedNearAuto extends NearAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }
}
