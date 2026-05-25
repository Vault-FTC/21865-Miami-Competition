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
 * Paste final values back into NearGateTwoPathsRed.java.
 * All coordinates are in Red-alliance space (already mirrored — no further flip needed).
 */
@Configurable
@TeleOp(name = "TUNE: Near Gate Two - Red", group = "TUNING - delete when done")
public class TuneNearGateTwoRed extends OpMode {

    // ── Paste final values back into NearGateTwoPathsRed.java ────────────────

    public static double START_X = 124,    START_Y = 127,   START_H = 40;

    // Shoot1 (Red space)
    public static double SH1_X0 = 124.919, SH1_Y0 = 124.005;
    public static double SH1_X1 = 95.874,  SH1_Y1 = 94.965;
    public static double SH1_H0 = 40,      SH1_H1 = 36.5;

    // SpikeIntake1 (Red space)
    public static double SP1_CX = 87.485,  SP1_CY = 56.865;
    public static double SP1_X1 = 138.051, SP1_Y1 = 59.220;

    // Shoot2 (Red space)
    public static double SH2_CX = 113.716, SH2_CY = 52.943;
    public static double SHOOT_X = 99,     SHOOT_Y = 95;

    // GateIntake1 (Red space)
    public static double GI1_CX = 74,    GI1_CY = 70;
    public static double GI1_X1 = 126.5, GI1_Y1 = 62;

    // GateIntake2 (Red space)
    public static double GI2_CX = 81, GI2_CY = 54;

    // Shoot4 (Red space)
    public static double SH4_CX = 81, SH4_CY = 80;

    // SpikeIntake2 (Red space)
    public static double SP2_CX = 90.714, SP2_CY = 27.325;
    public static double SP2_X1 = 129,    SP2_Y1 = 35;

    // Shoot5 (Red space)
    public static double SH5_CX = 95.028, SH5_CY = 57.951;

    // SpikeIntake3 (Red space)
    public static double SP3_CX = 105.707, SP3_CY = 81.262;
    public static double SP3_X1 = 130.485, SP3_Y1 = 83.696;

    // Shoot6 (Red space)
    public static double SH6_X1 = 87.153, SH6_Y1 = 104.725;
    public static double SH6_H1 = 30;

    // ── Internal ───────────────────────────────────────────────────────────────
    private static final Style SHOOT  = new Style("shoot",  "#FF4444", 1.2f);
    private static final Style INTAKE = new Style("intake", "#44DD44", 1.2f);
    private static final Style GATE   = new Style("gate",   "#4488FF", 1.2f);
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
                new BezierLine(new Pose(SH1_X0, SH1_Y0), new Pose(SH1_X1, SH1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H0), Math.toRadians(SH1_H1)).build();

        PathChain spikeIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SH1_X1, SH1_Y1), new Pose(SP1_CX, SP1_CY), new Pose(SP1_X1, SP1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(0)).build();

        PathChain shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SP1_X1, SP1_Y1), new Pose(SH2_CX, SH2_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(SH1_H1)).build();

        PathChain gateIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(GI1_CX, GI1_CY), new Pose(GI1_X1, GI1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(20)).build();

        PathChain shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(GI1_X1, GI1_Y1), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(SH1_H1)).build();

        PathChain gateIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(GI2_CX, GI2_CY), new Pose(GI1_X1, GI1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(20)).build();

        PathChain shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(GI1_X1, GI1_Y1), new Pose(SH4_CX, SH4_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(SH1_H1)).build();

        PathChain spikeIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(SP2_CX, SP2_CY), new Pose(SP2_X1, SP2_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(0)).build();

        PathChain shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SP2_X1, SP2_Y1), new Pose(SH5_CX, SH5_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(SH1_H1)).build();

        PathChain spikeIntake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(SP3_CX, SP3_CY), new Pose(SP3_X1, SP3_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(0)).build();

        PathChain shoot6 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SP3_X1, SP3_Y1), new Pose(SH6_X1, SH6_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(SH6_H1)).build();

        return new PathChain[]{
                shoot1, spikeIntake1, shoot2, gateIntake1, shoot3,
                gateIntake2, shoot4, spikeIntake2, shoot5, spikeIntake3, shoot6
        };
    }

    /** Builds paths and draws them for preview during init_loop. */
    private void buildAndDrawPreview() {
        PathChain[] preview = buildPaths();
        Drawing.drawRobot(new Pose(START_X, START_Y, Math.toRadians(START_H)), START_STYLE);
        Drawing.drawPath(preview[0],  SHOOT);   // shoot1
        Drawing.drawPath(preview[1],  INTAKE);  // spikeIntake1
        Drawing.drawPath(preview[2],  SHOOT);   // shoot2
        Drawing.drawPath(preview[3],  GATE);    // gateIntake1
        Drawing.drawPath(preview[4],  SHOOT);   // shoot3
        Drawing.drawPath(preview[5],  GATE);    // gateIntake2
        Drawing.drawPath(preview[6],  SHOOT);   // shoot4
        Drawing.drawPath(preview[7],  INTAKE);  // spikeIntake2
        Drawing.drawPath(preview[8],  SHOOT);   // shoot5
        Drawing.drawPath(preview[9],  INTAKE);  // spikeIntake3
        Drawing.drawPath(preview[10], SHOOT);   // shoot6
        Drawing.sendPacket();
    }
}
