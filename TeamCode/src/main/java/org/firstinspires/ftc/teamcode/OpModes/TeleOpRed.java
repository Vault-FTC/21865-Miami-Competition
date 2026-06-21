package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

@TeleOp(name = "TeleOp Red", group = "Teleop")
public class TeleOpRed extends TeleOpBlue {
   @Override public void setTargets()
    {
        limelight = new LimeLight(hardwareMap, 24, drivebase);
        gatePosition = new Location(337, 141.5, 21.4);
        goal = Constants.RED_CENTER_GOAL;
        shooter.setGoal(Constants.RED_CENTER_GOAL);
        headingOffset = Math.PI/2;
        parkPosition = new Location(411, -22.6,-90 );
        resetPointX = 19;
        resetPointY = 24;
        resetPointHeadingRadians = 0;
    }
}
