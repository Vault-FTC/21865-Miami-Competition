package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPathsRed;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.FarAuto;

@Autonomous(name = "Red Far Auto")
public class RedFarAuto extends FarAuto {
    @Override
    protected FarPaths createPaths(Follower follower) { return new FarPathsRed(follower); }
}
