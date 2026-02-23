package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

public class LimeLightTurnCommand extends Command
{
    Drivebase drivebase;
    LimeLight limeLight;
    Telemetry telemetry;
    ElapsedTime elapsedTime = new ElapsedTime();

    public LimeLightTurnCommand(Drivebase drivebase, LimeLight limeLight, Telemetry telemetry){
        this.drivebase = drivebase;
        this.limeLight = limeLight;
        this.telemetry = telemetry;
    }

    @Override
    public void execute() {
        LLResult res = limeLight.getResult();
        if(res == null)
        {
            drivebase.drive(0,0, 0);
            return;
        }
        Pose3D pose = res.getBotpose_MT2();
        double p = 0.05;
        double x = (0 + pose.getPosition().x) * p; // correct = 0;
        double z = (-1.8 + pose.getPosition().z) * p;  // correct = -1.8;
        double yaw = limeLight.getTx();
        drivebase.drive(0,0, yaw / 13);
        telemetry.addData("yaw", yaw);
    }

    @Override
    public void initialize() {
        elapsedTime.reset();
    }

    @Override
    public boolean isFinished(){
       return elapsedTime.seconds() > 100;

    }
}
