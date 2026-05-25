package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGateTwoPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGateTwoPathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearGateTwo;

@Autonomous(name = "Blue Near Gate Two")
public class BlueNearGateTwo extends NearGateTwo {
    @Override
    protected NearGateTwoPaths createPaths(Follower follower) { return new NearGateTwoPathsBlue(follower); }
}
