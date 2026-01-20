package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

public class LimeLightTurnCommand extends Command
{
    Drivebase moved;
    LimeLight light;
    Telemetry tele;
    ElapsedTime tenseistime = new ElapsedTime();

    public LimeLightTurnCommand(Drivebase move, LimeLight lime, Telemetry tele){
        moved = move;
        light = lime;
        this.tele = tele;
    }

    @Override
    public void execute() {
        LLResult res = light.getResult();
        if(res == null)
        {
            moved.drive(0,0, 0);
            return;
        }
        Pose3D pose = res.getBotpose_MT2();
        double p = 0.05;
        double x = (0 + pose.getPosition().x) * p; // correct = 0;
        double z = (-1.8 + pose.getPosition().z) * p;  // correct = -1.8;
        double yaw = light.getTx();
        moved.drive(0,0, yaw / 13);
        tele.addData("yaw", yaw);
    }

    @Override
    public void initialize() {
        tenseistime.reset();
    }

    @Override
    public boolean isFinished(){
       return tenseistime.seconds() > 100;

    }
}
