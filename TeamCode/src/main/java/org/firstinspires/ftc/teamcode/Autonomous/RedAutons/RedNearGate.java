package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearGate;

@Autonomous(name = "Red Near Gate")
public class RedNearGate extends NearGate {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }
}
