package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

@Autonomous(name = "Red Near", group = "Red Team")
public class RedNearAuto extends BaseNearAuto {

    @Override
    void setTargets()
    {
        LimeLight = new LimeLight(hardwareMap,24);
        launchPosition = new Location(-60, 60, 135);
        collectFirstRowArtifacts = new Location(-30.5, 124, 90);
//        hitGate = new Location(-60, 105, 180);
//        hitGate2 = new Location(-70, 110, 180);
        prepareSecondRowArtifacts = new Location(30,80, 90);
        collectSecondRowArtifacts = new Location(30, 135, 90);
        prepareThirdRowArtifacts = new Location(90,80, 90);
        collectThirdRowArtifacts = new Location(90,135, 90);
        lastLaunchPosition = new Location(-80, 60, 135);
        drive.setCurrentPose(new Pose2D(DistanceUnit.CM, -121.92, 121.92, AngleUnit.DEGREES, 135));
    }
}
