package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class VisionFixQuality {
    // ── Thresholds ─────────────────────────────────────────────────────────────
    public static final double MIN_TARGET_AREA  = 0.1;   // % of frame (was 0.5 — too strict for far tags)
    public static final double MAX_TARGET_AREA  = 25.0;  // too close = distortion
    public static final double MAX_POSE_JUMP    = 75.0;  // cm  (was 50 — relaxed for startup uncertainty)
    public static final double MAX_HEADING_JUMP = 0.5;   // radians (~28°)
    public static final double MAX_LATENCY_MS   = 200.0; // ms  (was 100 — MegaTag2 can take 120–180 ms)
    // ───────────────────────────────────────────────────────────────────────────

    /**
     * Field-corner-to-centre offset: pedro_cm = limelight_m × 100 + HALF_FIELD_CM.
     */
    private static final double HALF_FIELD_CM = 72.0 * 2.54; // 182.88 cm

    // ── Diagnostic state (updated every isGoodFix() call) ─────────────────────
    /** Human-readable reason the last fix was rejected, or "OK" if accepted. */
    private String lastRejectReason = "no result yet";
    /** Raw values captured on the last call (NaN if not available). */
    private double lastTa      = Double.NaN;
    private double lastLatency = Double.NaN;
    private double lastJumpCm  = Double.NaN;
    private double lastHdgDiff = Double.NaN;
    // ───────────────────────────────────────────────────────────────────────────

    public boolean isGoodFix(LLResult result, Pose2D currentPose) {
        // Reset diagnostics.
        lastTa = lastLatency = lastJumpCm = lastHdgDiff = Double.NaN;

        if (result == null || !result.isValid()) {
            lastRejectReason = "invalid result";
            return false;
        }

        lastTa = result.getTa();
        if (lastTa < MIN_TARGET_AREA) {
            lastRejectReason = String.format("ta too small (%.2f%% < %.2f%%)", lastTa, MIN_TARGET_AREA);
            return false;
        }
        if (lastTa > MAX_TARGET_AREA) {
            lastRejectReason = String.format("ta too large (%.2f%% > %.2f%%)", lastTa, MAX_TARGET_AREA);
            return false;
        }

        lastLatency = result.getTargetingLatency() + result.getCaptureLatency();
        if (lastLatency > MAX_LATENCY_MS) {
            lastRejectReason = String.format("latency %.0f ms > %.0f ms", lastLatency, MAX_LATENCY_MS);
            return false;
        }

        Pose3D botPose = result.getBotpose_MT2();
        if (botPose == null) {
            lastRejectReason = "no MT2 pose";
            return false;
        }

        double visionX = botPose.getPosition().x * 100.0 + HALF_FIELD_CM;
        double visionY = botPose.getPosition().y * 100.0 + HALF_FIELD_CM;
        double dx = visionX - currentPose.getX(DistanceUnit.CM);
        double dy = visionY - currentPose.getY(DistanceUnit.CM);
        lastJumpCm = Math.hypot(dx, dy);
        if (lastJumpCm > MAX_POSE_JUMP) {
            lastRejectReason = String.format("pos jump %.1f cm > %.1f cm", lastJumpCm, MAX_POSE_JUMP);
            return false;
        }

        // Limelight yaw is CCW-positive; Pinpoint heading is CW-positive.
        // Negate vision yaw to convert to CW before comparing.
        double visionHeading = -botPose.getOrientation().getYaw(AngleUnit.RADIANS); // CCW → CW
        double odoHeading    =  currentPose.getHeading(AngleUnit.RADIANS);          // CW+
        lastHdgDiff = Math.abs(normalize(visionHeading - odoHeading));
        if (lastHdgDiff > MAX_HEADING_JUMP) {
            lastRejectReason = String.format("hdg diff %.1f° > %.1f°",
                    Math.toDegrees(lastHdgDiff), Math.toDegrees(MAX_HEADING_JUMP));
            return false;
        }

        lastRejectReason = "OK";
        return true;
    }

    /** Returns a short description of why the last fix was rejected (or "OK"). */
    public String getLastRejectReason() { return lastRejectReason; }

    /** Raw target-area % from the last call (NaN if result was null/invalid). */
    public double getLastTa()      { return lastTa; }
    /** Combined latency ms from the last call. */
    public double getLastLatency() { return lastLatency; }
    /** Position jump cm from the last call (NaN if no MT2 pose). */
    public double getLastJumpCm()  { return lastJumpCm; }
    /** Heading difference radians from the last call (NaN if no MT2 pose). */
    public double getLastHdgDiff() { return lastHdgDiff; }

    private double normalize(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
