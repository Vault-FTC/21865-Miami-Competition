package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AutonomousPositions {
    //BLUE
    public static final Location launchPosition = new Location(-60, -60, -135);
    public static final Location collectFirstRowArtifacts = new Location(-30.5, -140.4, -90);
    public static final Location hitGate = new Location(-60, -105, -180);
    public static final Location hitGate2 = new Location(-70, -110, -180);
    public static final Location prepareSecondRowArtifacts = new Location(30,-80, -90);
    public static final Location collectSecondRowArtifacts = new Location(30, -135, -90);
    public static final Location prepareCollectThirdRowArtifacts = new Location(90,-80, -90);
    public static final Location collectionThirdRowArtifacts = new Location(90,-145, -90);
    public static final Location lastLaunchPosition = new Location(-80, -60, -125);
    public static final Location leaveZonePosition = new Location(0, -70, -90);
    public static final Pose2D BLUE_START_POSITION = new Pose2D(DistanceUnit.CM, -133, -133, AngleUnit.DEGREES, -135);
    public static final Pose2D RED_START_POSITION = new Pose2D(DistanceUnit.CM, -121.92, 121.92, AngleUnit.RADIANS, 0.785398);
    public static final Pose2D BLUE_FAR_POSITION = new Pose2D(DistanceUnit.CM, 152.4, -15.25, AngleUnit.DEGREES, -180);
    public static final Pose2D RED_FAR_POSITION = new Pose2D(DistanceUnit.CM, 152.4, 15.25, AngleUnit.DEGREES, -180);

    //RED

}
