package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;

public class DriveToCommandDynamicAim extends Command {

    public interface PathSupplier {
        ShootPathBuilder.Result build();
    }

    private static final double HEADING_TOLERANCE = Math.toRadians(3);

    private final Follower follower;
    private final PathSupplier supplier;
    private final double timeout;
    private final Telemetry telemetry;
    private final ElapsedTime elapsedTime = new ElapsedTime();

    private Pose targetPose;
    private boolean pathStarted;

    public DriveToCommandDynamicAim(Follower follower, PathSupplier supplier,
                                     double timeout, Telemetry telemetry) {
        this.follower  = follower;
        this.supplier  = supplier;
        this.timeout   = timeout;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        ShootPathBuilder.Result result = supplier.build();
        targetPose  = result.endPose;
        pathStarted = false;
        follower.followPath(result.path);
        elapsedTime.reset();
    }

    @Override
    public void execute() {
        if (follower.isBusy()) {
            pathStarted = true;
        } else if (pathStarted) {
            follower.holdPoint(targetPose);
        }
        follower.update();
        telemetry.addData("Running", "DynamicAim DriveTo");
        telemetry.addData("Heading error (deg)",
                Math.toDegrees(Math.abs(angleWrap(follower.getPose().getHeading() - targetPose.getHeading()))));
    }

    @Override
    public boolean isFinished() {
        double error = Math.abs(angleWrap(follower.getPose().getHeading() - targetPose.getHeading()));
        return (pathStarted && !follower.isBusy() && error < HEADING_TOLERANCE) || elapsedTime.seconds() >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }

    private double angleWrap(double r) {
        while (r > Math.PI)  r -= 2 * Math.PI;
        while (r < -Math.PI) r += 2 * Math.PI;
        return r;
    }
}
