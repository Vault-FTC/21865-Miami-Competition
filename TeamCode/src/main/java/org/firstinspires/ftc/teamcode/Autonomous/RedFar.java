package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;


@Autonomous(name = "Red Far", group = "Red Team")
public class RedFar extends BaseFarAuto {
    @Override
    void setTargets() {
        LimeLight = new LimeLight(hardwareMap,24);

        startPosition = AutonomousPositions.RED_FAR_POSITION;

        doNotHitWall = new Location(152.4, 15.25, -180);
        farShootPosition = new Location(152, 15.25, -165);
        firstPickupPosition = new Location(90, 50, -90);
        firstPickupPosition2 = new Location(145, 90, -90);
        parkPosition = new Location(133, 60, 90);

//        doNotHitWall = new Location(30, 0, -20);
//        farShootPosition = new Location(145, 10, 160);
//        firstPickupPosition = new Location(80, 0, -95);
//        firstPickupPosition2 = new Location(70, 130, -95);
//        parkPosition = new Location(133, -60, -90);
// ADB wifi
    }
}


