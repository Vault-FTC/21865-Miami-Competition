package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Abstract base for near-side 6-shot spike/gate paths. Extend with {@link NearGateTwoPathsBlue} or {@link NearGateTwoPathsRed}. */
public abstract class NearGateTwoPaths {

    public PathChain Shoot1, SpikeIntake1, Shoot2, GateIntake1, Shoot3,
                     GateIntake2, Shoot4, SpikeIntake2, Shoot5, SpikeIntake3, Shoot6;

    protected Pose startingPose;

    public Pose getStartingPose() { return startingPose; }

    public abstract ShootPathBuilder.Result buildShoot1();
    public abstract ShootPathBuilder.Result buildShoot2();
    public abstract ShootPathBuilder.Result buildShoot3();
    public abstract ShootPathBuilder.Result buildShoot4();
    public abstract ShootPathBuilder.Result buildShoot5();
    public abstract ShootPathBuilder.Result buildShoot6();
}
