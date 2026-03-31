package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

@TeleOp(name = "TeleOp Red", group = "Teleop")
public class TeleOpRed extends TeleOpBlue {
   @Override public void setTargets()
    {
        limeLight = new LimeLight(hardwareMap, 24);
        gatePosition = new Location(57, -147, 123);
        goal = Constants.RED_CENTER_GOAL;
        headingOffset = Math.PI;
        parkPosition = new Location(411, -22.6,-90 );
    }
}
