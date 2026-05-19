package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.NearGate;

@Autonomous(name = "Blue Near Gate")
public class BlueNearGate extends NearGate {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }
}
