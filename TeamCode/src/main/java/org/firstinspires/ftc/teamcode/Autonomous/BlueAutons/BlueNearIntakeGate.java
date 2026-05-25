package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearIntakeGatePaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearIntakeGatePathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.NearIntakeGate;

@Autonomous(name = "Blue Near Intake Gate")
public class BlueNearIntakeGate extends NearIntakeGate {
    @Override
    protected NearIntakeGatePaths createPaths(Follower follower) { return new NearIntakeGatePathsBlue(follower); }
}
