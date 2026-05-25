package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/**
 * Abstract base for near-side tunnel autonomous paths.
 * Extend with {@link NearPathsBlue} or {@link NearPathsRed}.
 */
public abstract class NearPaths {

    public PathChain Shoot1, Intake1, Gate1, Gate1V2, Shoot2, Shoot2Gate,
                     Intake2, Gate2, Shoot3Gate, Intake3Gate, Shoot3, Intake3,
                     GateIntake3, GateShoot4, Shoot4, Park;

    protected Pose startingPose;

    public Pose getStartingPose() { return startingPose; }

    public abstract ShootPathBuilder.Result buildShoot1();
    public abstract ShootPathBuilder.Result buildShoot2();
    public abstract ShootPathBuilder.Result buildShoot3();
    public abstract ShootPathBuilder.Result buildShoot4();
    public abstract ShootPathBuilder.Result buildGateShoot4();
}
