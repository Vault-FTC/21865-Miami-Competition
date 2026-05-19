package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.NearGateTwo;

@Autonomous(name = "Red Near Gate Two")
public class RedNearGateTwo extends NearGateTwo {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }
}
