package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;

/**
 * Keeps the robot locked on the shoot heading using Pedro's holdPoint for the
 * specified duration. Pair this with TimedShootCommand in a ParallelCommandGroup
 * so the heading is actively corrected throughout the entire shot.
 *
 * Pass the same PathSupplier used by the preceding DriveToCommandDynamicAim and
 * the same duration used by TimedShootCommand so both commands finish together.
 */
public class HoldHeadingCommand extends Command {

    private final Follower follower;
    private final DriveToCommandDynamicAim.PathSupplier supplier;
    private final double durationSeconds;
    private final ElapsedTime holdTimer = new ElapsedTime();

    private Pose targetPose;

    public HoldHeadingCommand(Follower follower,
                               DriveToCommandDynamicAim.PathSupplier supplier,
                               double durationSeconds) {
        this.follower        = follower;
        this.supplier        = supplier;
        this.durationSeconds = durationSeconds;
    }

    @Override
    public void initialize() {
        // Re-evaluate the path supplier to get the target end pose.
        // We only need the heading, so building the path again is fine.
        ShootPathBuilder.Result result = supplier.build();
        targetPose = result.endPose;
        holdTimer.reset();
    }

    @Override
    public void execute() {
        follower.holdPoint(targetPose);
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return holdTimer.seconds() >= durationSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }
}
