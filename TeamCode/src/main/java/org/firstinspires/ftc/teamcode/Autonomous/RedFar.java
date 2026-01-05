package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;


@Autonomous(name = "Red Far", group = "Red Team")
public class RedFar extends BaseFarAuto {
    @Override
    void setTargets() {
        LimeLight = new LimeLight(hardwareMap,24);
        doNotHitWall = new Location(30, 0, -20);
        farShootPosition = new Location(30, 0, -20);
        firstPickupPosition = new Location(80, 0, -95);
        firstPickupPosition2 = new Location(70, 130, -95);
        parkPosition = new Location(88, 0, -95);

    }
}


