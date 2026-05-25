package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearIntakeGatePaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearIntakeGatePathsRed;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearIntakeGate;

@Autonomous(name = "Red Near Intake Gate")
public class RedNearIntakeGate extends NearIntakeGate {
    @Override
    protected NearIntakeGatePaths createPaths(Follower follower) { return new NearIntakeGatePathsRed(follower); }
}
