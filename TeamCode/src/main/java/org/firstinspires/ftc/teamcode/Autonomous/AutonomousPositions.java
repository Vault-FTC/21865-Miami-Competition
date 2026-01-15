package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AutonomousPositions {
    //BLUE
    public static final Location launchPosition = new Location(30.5, -30.5, 0.785398);
    public static final Location collectFirstRowArtifacts = new Location(30.5, -152.4, Math.PI);
    public static final Location hitGate = new Location(-60, -105, -48);
    public static final Location hitGate2 = new Location(-70, -110, -48);
    public static final Location prepareSecondRowArtifacts = new Location(-145,-80, 43);
    public static final Location collectSecondRowArtifacts = new Location(-88, -135, 43);
    public static final Location prepareCollectThirdRowArtifacts = new Location(-184,-124, 43);
    public static final Location collectionThirdRowArtifacts = new Location(-124,-175, 43);
    public static final Location lastLaunchPosition = new Location(-110, 20, 0);
    public static final Location leaveZonePosition = new Location(-80, -70, 43);
    public static final Pose2D BLUE_START_POSITION = new Pose2D(DistanceUnit.CM, 121.92, -121.92, AngleUnit.RADIANS, 0.785398);
    public static final Pose2D RED_START_POSITION = new Pose2D(DistanceUnit.CM, 121.92, 121.92, AngleUnit.RADIANS, 0.785398);

    //RED

}
