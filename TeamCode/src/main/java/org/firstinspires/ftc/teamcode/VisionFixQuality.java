package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class VisionFixQuality {
    public static final double MIN_TARGET_AREA = 0.5; // % of frame
    public static final double MAX_TARGET_AREA = 25.0; // too close = distortion
    public static final int MIN_TAGS_FOR_MT2 = 2; // Multi-tag preferred
    public final double MAX_POSE_JUMP = 0.5; // meters - reject outliers
    public final double MAX_HEADING_JUMP = 0.5; // meters - reject outliers
    public final double MAX_LATENCY_MS = 100; // Stale data threshold

    public boolean isGoodFix(LLResult result, Pose2D currentPose) {
        if (result == null || !result.isValid()) return false;

        double ta = result.getTa();
        if (ta < MIN_TARGET_AREA || ta > MAX_TARGET_AREA) return false;

        double latency = result.getTargetingLatency() + result.getCaptureLatency();

        if (latency > MAX_LATENCY_MS) return false;

        Pose3D botPose = result.getBotpose_MT2();
        if (botPose == null) return false;

        double dx = botPose.getPosition().x - currentPose.getX(DistanceUnit.CM);
        double dy = botPose.getPosition().y - currentPose.getY(DistanceUnit.CM);
        double jump = Math.hypot(dx, dy);

        if (jump > MAX_POSE_JUMP) return false;

        double visionHeading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);
        double odoHeading = currentPose.getHeading(AngleUnit.RADIANS);

        double headingError = Math.abs(normalize(visionHeading - odoHeading));

        if (headingError > MAX_HEADING_JUMP) return false;

//        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
//        if (tags.size() < MIN_TAGS_FOR_MT2) return false;

        return true;
    }

    private double normalize(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
