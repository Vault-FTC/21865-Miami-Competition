package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.NearIntakeGate;

@Autonomous(name = "Red Near Intake Gate")
public class RedNearIntakeGate extends NearIntakeGate {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }
}
