package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.RootAutons.TestIntakePowerDraw;

@Autonomous (name = "BlueTestIntakePowerDraw")
public class BlueTestIntakePowerDraw extends TestIntakePowerDraw {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }
}
