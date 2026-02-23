package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;

public class PedroDriveToCommand extends Command {

    Telemetry telemetry;
    private final Follower follower;
    private final PathChain pathChain;
    double timeout;
    ElapsedTime elapsedTime = new ElapsedTime();

    public PedroDriveToCommand(Follower follower, PathChain pathChain, double timeout, Telemetry telemetry) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.timeout = timeout;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain);
        elapsedTime.reset();
    }

    @Override
    public void execute() {
        follower.update();
        telemetry.addData("Running", "Pedro DriveTo Command");
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy() || elapsedTime.seconds() >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }
}
