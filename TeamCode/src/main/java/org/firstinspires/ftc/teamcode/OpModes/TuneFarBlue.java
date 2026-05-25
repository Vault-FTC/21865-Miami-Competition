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
 * Paste final values back into FarPathsBlue.java.
 */
@Configurable
@TeleOp(name = "TUNE: Far - Blue", group = "TUNING - delete when done")
public class TuneFarBlue extends OpMode {

    // ── Paste final values back into FarPathsBlue.java ───────────────────────

    public static double START_X = 54,  START_Y = 8.5,   START_H = 90;

    // Shoot1 (line: START → end)
    public static double SH1_X1 = 57,  SH1_Y1 = 12;
    public static double SH1_H1 = 105;

    // Intake1 (curve: SH1 end → handle → end)
    public static double IN1_CX = 61.280, IN1_CY = 33;
    public static double IN1_X1 = 20,     IN1_Y1 = 33;

    // Shoot2 (line: IN1 end → SH1 end)
    public static double SH2_H1 = 129;

    // Intake2 (line: SH1 end → end)
    public static double IN2_X1 = 10,  IN2_Y1 = 5;

    // Shoot3
    public static double SH3_X0 = 10,    SH3_Y0 = 10;
    public static double SH3_X1 = 57,    SH3_Y1 = 11.951;
    public static double SH3_H1 = 129;

    // Intake3
    public static double IN3_X0 = 57,    IN3_Y0 = 11.971;
    public static double IN3_X1 = 3,     IN3_Y1 = 13.627;

    // Shoot4
    public static double SH4_X0 = 10,    SH4_Y0 = 23.627;

    // Park
    public static double PARK_X = 57,    PARK_Y = 31.364;
    public static double PARK_H = 90;

    // ── Internal ───────────────────────────────────────────────────────────────
    private static final Style SHOOT  = new Style("shoot",  "#FF4444", 1.2f);
    private static final Style INTAKE = new Style("intake", "#44DD44", 1.2f);
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
                new BezierLine(new Pose(START_X, START_Y), new Pose(SH1_X1, SH1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(START_H), Math.toRadians(SH1_H1)).build();

        PathChain intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SH1_X1, SH1_Y1), new Pose(IN1_CX, IN1_CY), new Pose(IN1_X1, IN1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(180)).build();

        PathChain shoot2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(IN1_X1, IN1_Y1), new Pose(SH1_X1, SH1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(SH2_H1)).build();

        PathChain intake2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SH1_X1, SH1_Y1), new Pose(IN2_X1, IN2_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH2_H1), Math.toRadians(180)).build();

        PathChain shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SH3_X0, SH3_Y0), new Pose(SH3_X1, SH3_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(SH3_H1)).build();

        PathChain intake3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(IN3_X0, IN3_Y0), new Pose(IN3_X1, IN3_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH3_H1), Math.toRadians(180)).build();

        PathChain shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SH4_X0, SH4_Y0), new Pose(SH3_X1, SH3_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(SH3_H1)).build();

        PathChain park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SH3_X1, SH3_Y1), new Pose(PARK_X, PARK_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(SH3_H1), Math.toRadians(PARK_H)).build();

        return new PathChain[]{ shoot1, intake1, shoot2, intake2, shoot3, intake3, shoot4, park };
    }

    /** Builds paths and draws them for preview during init_loop. */
    private void buildAndDrawPreview() {
        PathChain[] preview = buildPaths();
        Drawing.drawRobot(new Pose(START_X, START_Y, Math.toRadians(START_H)), START_STYLE);
        Drawing.drawPath(preview[0], SHOOT);   // shoot1
        Drawing.drawPath(preview[1], INTAKE);  // intake1
        Drawing.drawPath(preview[2], SHOOT);   // shoot2
        Drawing.drawPath(preview[3], INTAKE);  // intake2
        Drawing.drawPath(preview[4], SHOOT);   // shoot3
        Drawing.drawPath(preview[5], INTAKE);  // intake3
        Drawing.drawPath(preview[6], SHOOT);   // shoot4
        Drawing.drawPath(preview[7], PARK);    // park
        Drawing.sendPacket();
    }
}
