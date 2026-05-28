package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPathsRed;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearTunnel;

@Disabled
@Autonomous(name = "Red Near Tunnel")
public class RedNearTunnel extends NearTunnel {
    @Override
    protected NearPaths createPaths(Follower follower) { return new NearPathsRed(follower); }
}
