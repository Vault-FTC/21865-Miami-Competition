package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPathsRed;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearAuto;

@Autonomous(name = "Red Near Auto")
public class RedNearAuto extends NearAuto {
    @Override
    protected NearPaths createPaths(Follower follower) { return new NearPathsRed(follower); }
}
