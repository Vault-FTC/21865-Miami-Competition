package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

@TeleOp(name = "TeleOp Red", group = "Teleop")
public class teleopred extends SimpleFieldCentricDrive {
   @Override public void setTargets()
    {
        Limelight = new LimeLight(hardwareMap, 24);
    }
}
