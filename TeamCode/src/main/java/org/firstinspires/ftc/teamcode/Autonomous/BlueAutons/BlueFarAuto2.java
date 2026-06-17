package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPaths2;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPaths2Blue;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.FarAuto2;

@Autonomous(name = "Blue Far Auto 2")
public class BlueFarAuto2 extends FarAuto2 {
    @Override
    protected FarPaths2 createPaths(Follower follower) { return new FarPaths2Blue(follower); }
}
