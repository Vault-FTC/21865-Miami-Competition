package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPaths2;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPaths2Red;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.FarAuto2;

@Autonomous(name = "Red Far Auto 2")
public class RedFarAuto2 extends FarAuto2 {
    @Override
    protected FarPaths2 createPaths(Follower follower) { return new FarPaths2Red(follower); }
}
