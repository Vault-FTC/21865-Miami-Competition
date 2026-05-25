package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Abstract base for near-side gate autonomous paths. Extend with {@link NearGatePathsBlue} or {@link NearGatePathsRed}. */
public abstract class NearGatePaths {

    public PathChain Shoot1, Intake1, Gate1V2, Shoot2, Shoot2Gate,
                     Intake2, Gate2, Shoot3Gate, Shoot3, Intake3, Shoot4, Park;

    protected Pose startingPose;

    public Pose getStartingPose() { return startingPose; }

    public abstract ShootPathBuilder.Result buildShoot1();
    public abstract ShootPathBuilder.Result buildShoot2Gate();
    public abstract ShootPathBuilder.Result buildShoot3Gate();
    public abstract ShootPathBuilder.Result buildShoot4();
}
