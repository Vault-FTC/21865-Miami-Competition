package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.FarAuto;

@Autonomous(name = "Red Far Auto")
public class RedFarAuto extends FarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }
}
