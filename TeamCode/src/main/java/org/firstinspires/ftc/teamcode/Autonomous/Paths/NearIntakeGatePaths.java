package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Abstract base for near-side 6-shot intake-gate paths. Extend with {@link NearIntakeGatePathsBlue} or {@link NearIntakeGatePathsRed}. */
public abstract class NearIntakeGatePaths {

    public PathChain Shoot1, Intake1, Shoot2, Intake2, Shoot3, Intake3,
                     Shoot4, Intake4, Shoot5, Intake5, Shoot6, Park;

    protected Pose startingPose;

    public Pose getStartingPose() { return startingPose; }

    public abstract ShootPathBuilder.Result buildShoot1();
    public abstract ShootPathBuilder.Result buildShoot2();
    public abstract ShootPathBuilder.Result buildShoot3();
    public abstract ShootPathBuilder.Result buildShoot4();
    public abstract ShootPathBuilder.Result buildShoot5();
    public abstract ShootPathBuilder.Result buildShoot6();
}
