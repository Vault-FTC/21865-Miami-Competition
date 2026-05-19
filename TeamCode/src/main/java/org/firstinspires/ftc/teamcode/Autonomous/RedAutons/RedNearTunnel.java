package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.NearTunnel;

@Autonomous(name = "Red Near Tunnel")
public class RedNearTunnel extends NearTunnel {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }
}
