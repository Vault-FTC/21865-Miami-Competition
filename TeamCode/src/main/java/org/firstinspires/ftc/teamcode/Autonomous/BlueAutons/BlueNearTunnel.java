package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.NearTunnel;

@Autonomous(name = "Blue Near Tunnel")
public class BlueNearTunnel extends NearTunnel {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }
}
