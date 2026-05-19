package org.firstinspires.ftc.teamcode.Autonomous.Paths;

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
 * Mirror rules:  x → 144 - x,  heading → π - heading,  y unchanged.
 *
 * Usage: define all paths in Blue coordinates, then call PathMirror.flip() on each path
 * inside the constructor when Alliance.RED is detected.
 */
public class PathMirror {

    private static final double FIELD_WIDTH = 144.0;

    /** Mirror a single Pose (x and heading flipped, y unchanged). */
    public static Pose mirrorPose(Pose pose) {
        return new Pose(
                FIELD_WIDTH - pose.getX(),
                pose.getY(),
                Math.PI - pose.getHeading()
        );
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

        Path newPath = new Path(newCurve);
        double startHeading = original.getHeadingGoal(0.0);
        double endHeading   = original.getHeadingGoal(1.0);
        newPath.setLinearHeadingInterpolation(Math.PI - startHeading, Math.PI - endHeading);
        return newPath;
    }
}
