package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

@TeleOp(name = "TeleOp Red", group = "Teleop")
public class teleopred extends SimpleFieldCentricDrive {
   @Override public void setTargets()
    {
        Limelight = new LimeLight(hardwareMap, 24);
        goal = FieldConstants.RED_CENTER_GOAL;
        headingOffset = Math.PI;
    }
}
