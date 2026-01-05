package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

@Autonomous(name = "Red Near", group = "Red Team")
public class RedNearAuto extends BaseNearAuto {

    @Override
    void setTargets()
    {
        LimeLight = new LimeLight(hardwareMap,24);
        launchPosition = new Location(-110, 0, 0);
        collectFirstRowArtifacts = new Location(-70, 95, -43);
        prepareSecondRowArtifacts = new Location(-152,80, -43);
        collectSecondRowArtifacts = new Location(-86, 140, -43);
        prepareCollectThirdRowArtifacts = new Location(-192,124, -43);
        collectionThirdRowArtifacts = new Location(-129,184, -43);
        leaveZonePosition = new Location(-80, 140, -43);
    }
}
