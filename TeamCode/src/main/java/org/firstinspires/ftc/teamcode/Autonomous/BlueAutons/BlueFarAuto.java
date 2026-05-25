package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.FarAuto;

@Autonomous(name = "Blue Far Auto")
public class BlueFarAuto extends FarAuto {
    @Override
    protected FarPaths createPaths(Follower follower) { return new FarPathsBlue(follower); }
}
