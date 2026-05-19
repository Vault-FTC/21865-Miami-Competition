package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearAuto;

@Autonomous(name = "Blue Near Auto")
public class BlueNearAuto extends NearAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }
}
