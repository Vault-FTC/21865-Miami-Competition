package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGatePaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGatePathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearGate;

@Autonomous(name = "Blue Near Gate")
public class BlueNearGate extends NearGate {
    @Override
    protected NearGatePaths createPaths(Follower follower) { return new NearGatePathsBlue(follower); }
}
