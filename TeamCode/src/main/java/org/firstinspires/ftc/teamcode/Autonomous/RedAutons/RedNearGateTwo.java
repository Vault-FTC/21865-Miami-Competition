package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGateTwoPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGateTwoPathsRed;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearGateTwo;

@Autonomous(name = "Red Near Gate Two")
public class RedNearGateTwo extends NearGateTwo {
    @Override
    protected NearGateTwoPaths createPaths(Follower follower) { return new NearGateTwoPathsRed(follower); }
}
