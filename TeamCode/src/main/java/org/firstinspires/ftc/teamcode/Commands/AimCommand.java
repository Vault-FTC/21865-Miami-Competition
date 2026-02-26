package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.OpModes.PedroTuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

public class AimCommand extends Command
{
    Drivebase drivebase;

    LimeLight limeLight;
    Telemetry telemetry;
    ElapsedTime elapsedTime = new ElapsedTime();

    private final Follower follower;
    private final double GOAL_X = -152.4;
    private final double GOAL_Y = -152.4;

    public AimCommand(Drivebase drivebase, LimeLight limeLight, Telemetry telemetry, Follower follower){
        this.drivebase = drivebase;
        this.limeLight = limeLight;
        this.telemetry = telemetry;
        this.follower = follower;
    }

    @Override
    public void execute() {
        LLResult res = limeLight.getResult();
//        if(res == null)
//        {
//            drivebase.drive(0,0, 0);
//            return;
//        }
//        Pose3D pose = res.getBotpose_MT2();
//        double p = 0.05;
//        double x = (0 + pose.getPosition().x) * p; // correct = 0;
//        double z = (-1.8 + pose.getPosition().z) * p;  // correct = -1.8;
//        double yaw = limeLight.getTx();
//        drivebase.drive(0,0, yaw / 13);
//        telemetry.addData("yaw", yaw);

        Pose pose = follower.getPose();

        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();

        double targetHeading = Math.atan2(dy, dx);

        if (res != null) {
            double tx = limeLight.getTx();
            targetHeading -= Math.toRadians(tx);
        }
        follower.setHeading(targetHeading);
    }

    @Override
    public void initialize() {
        elapsedTime.reset();
    }

    @Override
    public boolean isFinished(){
       return elapsedTime.seconds() > 1;
    }

}
