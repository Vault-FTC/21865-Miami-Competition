package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AutonomousPositions {
    //BLUE
    public static final Location launchPosition = new Location(-30.5, -30.5, -135);
    public static final Location collectFirstRowArtifacts = new Location(-20.5, -152.4, -90);
    public static final Location hitGate = new Location(-60, -105, -180);
    public static final Location hitGate2 = new Location(-70, -110, -180);
    public static final Location prepareSecondRowArtifacts = new Location(30,-80, -90);
    public static final Location collectSecondRowArtifacts = new Location(30, -135, -90);
    public static final Location prepareCollectThirdRowArtifacts = new Location(-184,-124, 43);
    public static final Location collectionThirdRowArtifacts = new Location(-124,-175, 43);
    public static final Location lastLaunchPosition = new Location(-30.5, -30.5, 0);
    public static final Location leaveZonePosition = new Location(-80, -70, 43);
    public static final Pose2D BLUE_START_POSITION = new Pose2D(DistanceUnit.CM, -133, -133, AngleUnit.DEGREES, -135);
    public static final Pose2D RED_START_POSITION = new Pose2D(DistanceUnit.CM, -121.92, 121.92, AngleUnit.RADIANS, 0.785398);

    //RED

}
