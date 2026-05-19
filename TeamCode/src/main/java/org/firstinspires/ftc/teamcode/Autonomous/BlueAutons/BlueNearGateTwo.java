package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearGateTwo;

@Autonomous(name = "Blue Near Gate Two")
public class BlueNearGateTwo extends NearGateTwo {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }
}
