package org.firstinspires.ftc.teamcode.Commands;

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
    private final double GOAL_X;
    private final double GOAL_Y;

    public AimCommand(Drivebase drivebase, LimeLight limeLight, Telemetry telemetry, Follower follower, double goalx, double goaly){
        this.drivebase = drivebase;
        this.limeLight = limeLight;
        this.telemetry = telemetry;
        this.follower = follower;
        this.GOAL_X = goalx;
        this.GOAL_Y = goaly;
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
        double error = targetHeading - pose.getHeading();
        double wrappedError = angleWrap(error);
        drivebase.drive(0, 0, wrappedError * -0.9);

        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Error (deg)", Math.toDegrees(wrappedError));

        follower.update();

    }

    @Override
    public void initialize() {
        elapsedTime.reset();
    }

    @Override
    public boolean isFinished(){
       return elapsedTime.seconds() > 1;
    }

    @Override
    public void end(boolean interrupted) {
        telemetry.addLine("Aim ended");
        drivebase.drive(0,0,0);
    }
    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }


}
