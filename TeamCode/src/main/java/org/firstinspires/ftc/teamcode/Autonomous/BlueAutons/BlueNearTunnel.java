package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearTunnel;

@Autonomous(name = "Blue Near Tunnel")
public class BlueNearTunnel extends NearTunnel {
    @Override
    protected NearPaths createPaths(Follower follower) { return new NearPathsBlue(follower); }
}
