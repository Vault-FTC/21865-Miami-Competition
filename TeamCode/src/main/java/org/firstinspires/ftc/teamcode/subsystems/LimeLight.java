package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.VisionFixQuality;

public class LimeLight {
    /** Weight applied to each fresh vision fix (0 = odometry only, 1 = vision only). */
    public static final double VISION_WEIGHT = 0.5;

    /**
     * Half the FTC field width in centimetres.
     * Limelight's MegaTag2 pose uses the WPILib convention: origin at the field centre,
     * position in metres.  The Pinpoint (via Pedro) uses a corner-origin in centimetres.
     * Conversion:  pedro_cm = limelight_m × 100 + HALF_FIELD_CM
     */
    private static final double HALF_FIELD_CM = 72.0 * 2.54; // 182.88 cm

    Limelight3A limelight;
    VisionFixQuality visionFixQuality;
    Drivebase drivebase;
    int apriltag;

    // ── Last-update diagnostics (refreshed every update() call) ──────────────
    private boolean lastHadResult   = false; // any valid LL result this cycle?
    private boolean lastFixAccepted = false; // did it pass isGoodFix()?
    // Raw MegaTag2 values (field-centre metres / degrees) — NaN when no result.
    private double lastLLX   = Double.NaN;
    private double lastLLY   = Double.NaN;
    private double lastLLYaw = Double.NaN;
    // Converted Pedro/Pinpoint values (corner-origin cm / degrees).
    private double lastPedroX   = Double.NaN;
    private double lastPedroY   = Double.NaN;
    // ─────────────────────────────────────────────────────────────────────────

    public LimeLight(HardwareMap hardwareMap, int apriltag, Drivebase drivebase)
    {
        this.apriltag = apriltag;
        this.drivebase = drivebase;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.visionFixQuality = new VisionFixQuality();
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public LLResult getLatestValidResult() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        return result;
    }

    public void update() {
        GoBildaPinpointDriver odo = drivebase.getOdo();
        odo.update();

        // MegaTag2 needs the robot's current yaw in CCW-positive (WPILib convention).
        // This robot's Pinpoint/Pedro uses CW-positive, so we negate before passing it in.
        // If MegaTag2 starts placing the robot on the mirrored side of the field again,
        // remove the negation here and re-test.
        limelight.updateRobotOrientation(-odo.getHeading(AngleUnit.DEGREES));

        // Reset diagnostics each cycle.
        lastHadResult = lastFixAccepted = false;
        lastLLX = lastLLY = lastLLYaw = lastPedroX = lastPedroY = Double.NaN;

        LLResult result = getLatestValidResult();
        if (result == null) return;
        lastHadResult = true;

        Pose3D visionPose = result.getBotpose_MT2();
        if (visionPose == null) return;

        // Record raw Limelight values for telemetry.
        lastLLX   = visionPose.getPosition().x;
        lastLLY   = visionPose.getPosition().y;
        lastLLYaw = visionPose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Convert Limelight's centre-origin metres → Pedro/Pinpoint corner-origin cm.
        lastPedroX = lastLLX * 100.0 + HALF_FIELD_CM;
        lastPedroY = lastLLY * 100.0 + HALF_FIELD_CM;

        if (!visionFixQuality.isGoodFix(result, odo.getPosition())) return;
        lastFixAccepted = true;

        double newX = odo.getPosX(DistanceUnit.CM) * (1 - VISION_WEIGHT) + lastPedroX * VISION_WEIGHT;
        double newY = odo.getPosY(DistanceUnit.CM) * (1 - VISION_WEIGHT) + lastPedroY * VISION_WEIGHT;

        // Blend heading using shortest-arc interpolation to avoid wrap-around glitches.
        // Pinpoint heading is CW-positive; Limelight yaw is CCW-positive.
        // Negate the vision yaw to convert it to CW before blending.
        double odoH = odo.getHeading(AngleUnit.RADIANS);                          // CW+
        double visH = -visionPose.getOrientation().getYaw(AngleUnit.RADIANS);     // CCW → CW
        double newH = odoH + VISION_WEIGHT * normalizeAngle(visH - odoH);         // stays CW+

        drivebase.setCurrentPose(new Pose2D(DistanceUnit.CM, newX, newY, AngleUnit.RADIANS, newH));
    }

    /**
     * Adds MegaTag2 diagnostic lines to the driver-station telemetry.
     * Call this after {@link #update()} each loop, before {@code telemetry.update()}.
     *
     * <p>How to verify the pose is correct:
     * <ol>
     *   <li>Place the robot at a known field location (e.g. exactly on the starting tile).</li>
     *   <li>Compare "LL→Pedro X/Y" to the expected Pedro coordinates for that spot
     *       (near-side start ≈ X 51 cm, Y 323 cm).</li>
     *   <li>Face the robot squarely toward the Red alliance wall; "LL Yaw" should read ≈ 0°.</li>
     *   <li>If "LL→Pedro" matches but the heading sign is wrong (e.g. +90° instead of −90°),
     *       negate the heading in {@code updateRobotOrientation()} and re-test.</li>
     *   <li>If "LL→Pedro" is mirrored across the field centre (e.g. X reads 315 instead of 51),
     *       the X axis is inverted — negate {@code lastLLX} before adding {@code HALF_FIELD_CM}.</li>
     *   <li>If X and Y appear swapped, swap them in the conversion.</li>
     *   <li>"Fix accepted" must read YES for vision fusion to run at all.</li>
     * </ol>
     */
    public void addTelemetry(Telemetry t) {
        GoBildaPinpointDriver odo = drivebase.getOdo();

        t.addLine("── Limelight ──────────────────────");
        t.addData("  Result visible", lastHadResult  ? "YES" : "NO");
        t.addData("  Fix accepted",   lastFixAccepted ? "YES" : "NO");
        if (!lastFixAccepted) {
            // Show exactly which filter rejected the fix so you can tune thresholds.
            t.addData("  Reject reason", visionFixQuality.getLastRejectReason());
            if (!Double.isNaN(visionFixQuality.getLastTa()))
                t.addData("  ta (%)",        String.format("%.2f", visionFixQuality.getLastTa()));
            if (!Double.isNaN(visionFixQuality.getLastLatency()))
                t.addData("  latency (ms)",  String.format("%.0f", visionFixQuality.getLastLatency()));
            if (!Double.isNaN(visionFixQuality.getLastJumpCm()))
                t.addData("  pos jump (cm)", String.format("%.1f", visionFixQuality.getLastJumpCm()));
            if (!Double.isNaN(visionFixQuality.getLastHdgDiff()))
                t.addData("  hdg diff (°)",  String.format("%.1f", Math.toDegrees(visionFixQuality.getLastHdgDiff())));
        }

        if (lastHadResult && !Double.isNaN(lastLLX)) {
            // Raw MegaTag2 output (what the Limelight actually computed).
            t.addData("  LL raw X (m)",  String.format("%.3f", lastLLX));
            t.addData("  LL raw Y (m)",  String.format("%.3f", lastLLY));
            t.addData("  LL Yaw (°)",    String.format("%.1f", lastLLYaw));

            // Converted to Pedro space — this is what gets blended into odometry.
            t.addData("  LL→Pedro X (cm)", String.format("%.1f", lastPedroX));
            t.addData("  LL→Pedro Y (cm)", String.format("%.1f", lastPedroY));

            // Side-by-side heading comparison (both shown in CW-positive to match Pinpoint).
            // Limelight yaw is CCW-positive (WPILib); negate it for a fair comparison.
            double odoYaw   = odo.getHeading(AngleUnit.DEGREES);  // CW+
            double llYawCW  = -lastLLYaw;                          // CCW → CW
            t.addData("  Pinpoint hdg (°)", String.format("%.1f", odoYaw));
            t.addData("  LL yaw  (CW, °)",  String.format("%.1f", llYawCW));
            t.addData("  Hdg diff     (°)", String.format("%.1f", llYawCW - odoYaw));
        } else {
            t.addLine("  (no tag visible)");
        }
        t.addLine("────────────────────────────────────");
    }

    public Pose3D getBotPose() {
        LLResult r = getLatestValidResult();
        if (r == null) return null;
        return r.getBotpose_MT2();
    }

    /**
     * Returns the horizontal offset of the primary target from the crosshair, in degrees.
     * Positive = target is to the right of the crosshair.
     * Returns 0 if no valid target is visible.
     */
    public double getTx() {
        LLResult r = getLatestValidResult();
        if (r == null) return 0.0;
        return r.getTx(); // raw degrees — callers apply their own gain
    }

    public Pose3D getFieldPose() {
        LLResult result = getLatestValidResult();
        return result != null ? result.getBotpose_MT2() : null;
    }

    private double normalizeAngle(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
