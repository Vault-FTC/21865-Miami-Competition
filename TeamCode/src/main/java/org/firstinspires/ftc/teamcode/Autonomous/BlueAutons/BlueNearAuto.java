package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearAuto;

@Autonomous(name = "Blue Near Auto")
public class BlueNearAuto extends NearAuto {
    @Override
    protected NearPaths createPaths(Follower follower) { return new NearPathsBlue(follower); }
}
