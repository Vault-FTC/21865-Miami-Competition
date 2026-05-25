package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGatePaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGatePathsRed;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearGate;

@Autonomous(name = "Red Near Gate")
public class RedNearGate extends NearGate {
    @Override
    protected NearGatePaths createPaths(Follower follower) { return new NearGatePathsRed(follower); }
}
