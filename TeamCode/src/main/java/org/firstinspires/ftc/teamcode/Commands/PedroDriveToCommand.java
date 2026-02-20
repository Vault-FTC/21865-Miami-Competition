package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

public class PedroDriveToCommand extends Command {

    Telemetry telemetry;
    private final Follower follower;
    private final PathChain path;
    double timeout;
    ElapsedTime time = new ElapsedTime();

    public PedroDriveToCommand(Follower follower, PathChain path, double timeout) {
        this.follower = follower;
        this.path = path;
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        follower.followPath(path);
        time.reset();
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy() || time.milliseconds() >= timeout;
    }
}
