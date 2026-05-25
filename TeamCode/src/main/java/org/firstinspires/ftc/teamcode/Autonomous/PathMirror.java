package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

/**
 * Mirrors a PathChain (and Poses) from Blue alliance space to Red alliance space.
 *
 * Mirror rules:  x → FIELD_WIDTH - x,  heading → π - heading,  y unchanged.
 * All mirroring is delegated to {@link Pose#mirror(double)} so that PedroPathing's
 * coordinate-system conversion is applied before the transform — avoiding the bug
 * where getX()/getY() on a non-Pedro-coordinate Pose returns the wrong axis values.
 *
 * Usage: define all paths in Blue coordinates, then call PathMirror.flip() on each path
 * inside the constructor when Alliance.RED is detected.
 */
public class PathMirror {

    /**
     * FTC field width in inches.  PedroPathing's own Pose.mirror() defaults to 141.5;
     * we use 144.0 (12 ft) to match the coordinates used in our path files.
     */
    private static final double FIELD_WIDTH = 144.0;

    /**
     * Mirror a single Pose (x and heading flipped, y unchanged).
     * Delegates to {@link Pose#mirror(double)} so coordinate-system conversion
     * is handled correctly by PedroPathing before the transform is applied.
     */
    public static Pose mirrorPose(Pose pose) {
        return pose.mirror(FIELD_WIDTH);
    }

    /** Mirror all paths in a PathChain and return a new PathChain. */
    public static PathChain flip(PathChain chain) {
        ArrayList<Path> mirrored = new ArrayList<>();
        for (int i = 0; i < chain.size(); i++) {
            mirrored.add(flipPath(chain.getPath(i)));
        }
        return new PathChain(mirrored);
    }

    private static Path flipPath(Path original) {
        ArrayList<Pose> pts = original.getControlPoints();

        Curve newCurve;
        if (pts.size() == 2) {
            newCurve = new BezierLine(mirrorPose(pts.get(0)), mirrorPose(pts.get(1)));
        } else {
            List<Pose> mirroredPts = new ArrayList<>();
            for (Pose p : pts) {
                mirroredPts.add(mirrorPose(p));
            }
            newCurve = new BezierCurve(mirroredPts);
        }

        // Mirror the heading interpolation endpoints via mirrorPose so the same
        // coordinate-system-aware heading conversion is applied consistently.
        double startHeading = mirrorPose(new Pose(0, 0, original.getHeadingGoal(0.0))).getHeading();
        double endHeading   = mirrorPose(new Pose(0, 0, original.getHeadingGoal(1.0))).getHeading();

        Path newPath = new Path(newCurve);
        newPath.setLinearHeadingInterpolation(startHeading, endHeading);
        return newPath;
    }
}
