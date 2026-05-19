package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;

public class DriveToCommandDynamicAim extends Command {
    private final Follower follower;
    private final double goalX, goalY, timeout;
    private final Telemetry telemetry;
    private final Pose[] waypoints; // Same points we'd put into the Bezier line/Curve inside the Paths we build... unfortunately this will bring in the nasty paths into our autonomous file... sooo yeah
    private ElapsedTime elapsedTime = new ElapsedTime();

    public DriveToCommandDynamicAim(Follower follower, double goalX, double goalY, double timeout, Telemetry telemetry, Pose[] waypoints) {
        this.follower = follower;
        this.goalX = goalX;
        this.goalY = goalY;
        this.timeout = timeout;
        this.telemetry = telemetry;
        this.waypoints = waypoints;
    }

    @Override
    public void initialize() {
        double startHeading = follower.getPose().getHeading();

        Pose endpoint = waypoints[waypoints.length - 1];
        double endHeading = Math.atan2(goalY - endpoint.getY(), goalX - endpoint.getX());

        PathBuilder builder = follower.pathBuilder();

        if (waypoints.length == 2) {
            builder.addPath(new BezierLine(waypoints[0], waypoints[1]));
        } else {
            builder.addPath(new BezierCurve(waypoints));
        }

        follower.followPath(
                builder.setLinearHeadingInterpolation(startHeading, endHeading).build()
        );
        elapsedTime.reset();
    }

    @Override
    public void execute() {
        follower.update();
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
