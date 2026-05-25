package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * DELETE THIS FILE when tuning is done.
 * Paste final values back into NearGatePathsRed.java.
 * All coordinates are in Red-alliance space (already mirrored — no further flip needed).
 */
@Configurable
@TeleOp(name = "TUNE: Near Gate - Red", group = "TUNING - delete when done")
public class TuneNearGateRed extends OpMode {

    // ── Paste final values back into NearGatePathsRed.java ───────────────────

    public static double START_X = 124,   START_Y = 127,  START_H = 40;

    public static double SHOOT_X = 99,    SHOOT_Y = 95;
    public static double INTAKE1_X = 127, INTAKE1_Y = 84;
    public static double GATE_X = 135,    GATE_Y = 78;
    public static double INTAKE2_X = 129, INTAKE2_Y = 60;
    public static double INTAKE3_X = 119, INTAKE3_Y = 36;
    public static double PARK_X = 109,    PARK_Y = 90;

    // Gate1V2 (Red space)
    public static double G1V2_X0 = 129, G1V2_Y0 = 86;
    public static double G1V2_CX = 105, G1V2_CY = 85;
    public static double G1V2_X1 = 132, G1V2_Y1 = 74;

    // Shoot2Gate start (Red space)
    public static double SH2G_X0 = 132, SH2G_Y0 = 78;

    // Bezier handles (Red space)
    public static double INT1_CX = 76,   INT1_CY = 78;
    public static double INT2_CX = 81,   INT2_CY = 54;
    public static double GATE2_CX = 94,  GATE2_CY = 60;
    public static double INT3_CX = 81,   INT3_CY = 28;
    public static double SH3_CX = 121,   SH3_CY = 60;

    // Headings (Red-space mirrored)
    public static double H_SHOOT1_END  = 43;
    public static double H_SHOOT2G_END = 50;
    public static double H_SHOOT3G_END = 45;
    public static double H_SHOOT3_END  = 45;
    public static double H_SHOOT4_END  = 43;
    public static double H_PARK        = 290;

    // ── Internal ───────────────────────────────────────────────────────────────
    private static final Style SHOOT  = new Style("shoot",  "#FF4444", 1.2f);
    private static final Style INTAKE = new Style("intake", "#44DD44", 1.2f);
    private static final Style GATE   = new Style("gate",   "#4488FF", 1.2f);
    private static final Style PARK   = new Style("park",   "#FFFF44", 1.2f);
    private static final Style START_STYLE = new Style("start", "#FFFFFF", 1.5f);

    private Follower follower;
    private PathChain[] paths;
    private int pathIndex = -1;

    @Override
    public void init() {
        follower = Constants.PedroPathing.createFollower(hardwareMap);
        Drawing.init();
        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    @Override
    public void init_loop() {
        buildAndDrawPreview();
    }

    @Override
    public void start() {
        paths = buildPaths();
        pathIndex = 0;
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_H)));
        follower.followPath(paths[0], true);
    }

    @Override
    public void loop() {
        follower.update();

        if (pathIndex >= 0 && !follower.isBusy()) {
            pathIndex++;
            if (pathIndex < paths.length) {
                follower.followPath(paths[pathIndex], true);
            } else {
                pathIndex = -1;
            }
        }

        Drawing.drawDebug(follower);
        telemetry.addData("Status", pathIndex < 0 ? "Done — Stop/Init to re-run" : "Path " + (pathIndex + 1) + " / " + paths.length);
        telemetry.update();
    }

    /** Builds all PathChains from current static field values. */
    private PathChain[] buildPaths() {
        PathChain shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(START_X, START_Y), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(START_H), Math.toRadians(H_SHOOT1_END)).build();

        PathChain intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(INT1_CX, INT1_CY), new Pose(INTAKE1_X, INTAKE1_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(H_SHOOT1_END), Math.toRadians(0)).build();

        PathChain gate1v2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(G1V2_X0, G1V2_Y0), new Pose(G1V2_CX, G1V2_CY), new Pose(G1V2_X1, G1V2_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90)).build();

        PathChain shoot2gate = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SH2G_X0, SH2G_Y0), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(H_SHOOT2G_END)).build();

        PathChain intake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(INT2_CX, INT2_CY), new Pose(INTAKE2_X, INTAKE2_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(H_SHOOT2G_END), Math.toRadians(0)).build();

        PathChain gate2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(INTAKE2_X, INTAKE2_Y), new Pose(GATE2_CX, GATE2_CY), new Pose(GATE_X, GATE_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90)).build();

        PathChain shoot3gate = follower.pathBuilder().addPath(
                new BezierLine(new Pose(GATE_X, GATE_Y), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(H_SHOOT3G_END)).build();

        PathChain intake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(INT3_CX, INT3_CY), new Pose(INTAKE3_X, INTAKE3_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(H_SHOOT3G_END), Math.toRadians(0)).build();

        PathChain shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(INTAKE3_X, INTAKE3_Y), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(H_SHOOT4_END)).build();

        PathChain park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SHOOT_X, SHOOT_Y), new Pose(PARK_X, PARK_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(H_SHOOT4_END), Math.toRadians(H_PARK)).build();

        return new PathChain[]{
                shoot1, intake1, gate1v2, shoot2gate, intake2, gate2,
                shoot3gate, intake3, shoot4, park
        };
    }

    /** Builds paths and draws them for preview during init_loop. */
    private void buildAndDrawPreview() {
        PathChain[] preview = buildPaths();
        Drawing.drawRobot(new Pose(START_X, START_Y, Math.toRadians(START_H)), START_STYLE);
        Drawing.drawPath(preview[0], SHOOT);   // shoot1
        Drawing.drawPath(preview[1], INTAKE);  // intake1
        Drawing.drawPath(preview[2], GATE);    // gate1v2
        Drawing.drawPath(preview[3], SHOOT);   // shoot2gate
        Drawing.drawPath(preview[4], INTAKE);  // intake2
        Drawing.drawPath(preview[5], GATE);    // gate2
        Drawing.drawPath(preview[6], SHOOT);   // shoot3gate
        Drawing.drawPath(preview[7], INTAKE);  // intake3
        Drawing.drawPath(preview[8], SHOOT);   // shoot4
        Drawing.drawPath(preview[9], PARK);    // park
        Drawing.sendPacket();
    }
}
