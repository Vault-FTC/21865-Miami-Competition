package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Autonomous.ShootPathBuilder;

/** Abstract base for far-side autonomous paths. Extend with {@link FarPathsBlue} or {@link FarPathsRed}. */
public abstract class FarPaths {

    public PathChain Shoot1, Intake1, Shoot2, Intake2, Shoot3, Intake3, Shoot4, Park;

    protected Pose startingPose;

    public Pose getStartingPose() { return startingPose; }

    public abstract ShootPathBuilder.Result buildShoot1();
    public abstract ShootPathBuilder.Result buildShoot2();
    public abstract ShootPathBuilder.Result buildShoot3();
    public abstract ShootPathBuilder.Result buildShoot4();
}
