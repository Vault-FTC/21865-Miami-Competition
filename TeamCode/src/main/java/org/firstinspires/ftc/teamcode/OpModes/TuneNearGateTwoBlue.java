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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;

/**
 * DELETE THIS FILE when tuning is done.
 * Paste final values back into NearGateTwoPathsBlue.java.
 */
@Configurable
@TeleOp(name = "TUNE: Near Gate Two - Blue", group = "TUNING - delete when done")
public class TuneNearGateTwoBlue extends OpMode {

    // ── Paste final values back into NearGateTwoPathsBlue.java ───────────────

    public static double START_X = 20,    START_Y = 127,   START_H = 140;

    // Shoot1
    public static double SH1_X0 = 19.081, SH1_Y0 = 124.005;
    public static double SH1_X1 = 48.126, SH1_Y1 = 94.965;
    public static double SH1_H0 = 140,    SH1_H1 = 143.5;

    // SpikeIntake1
    public static double SP1_CX = 56.515, SP1_CY = 56.865;
    public static double SP1_X1 = 5.949,  SP1_Y1 = 59.220;

    // Shoot2
    public static double SH2_CX = 30.284, SH2_CY = 52.943;
    public static double SHOOT_X = 45,    SHOOT_Y = 95;

    // GateIntake1
    public static double GI1_CX = 70,   GI1_CY = 70;
    public static double GI1_X1 = 17.5, GI1_Y1 = 62;

    // GateIntake2
    public static double GI2_CX = 63, GI2_CY = 54;

    // Shoot4
    public static double SH4_CX = 63, SH4_CY = 80;

    // SpikeIntake2
    public static double SP2_CX = 53.286, SP2_CY = 27.325;
    public static double SP2_X1 = 15,     SP2_Y1 = 35;

    // Shoot5
    public static double SH5_CX = 48.972, SH5_CY = 57.951;

    // SpikeIntake3
    public static double SP3_CX = 38.293, SP3_CY = 81.262;
    public static double SP3_X1 = 13.515, SP3_Y1 = 83.696;

    // Shoot6
    public static double SH6_X1 = 56.847, SH6_Y1 = 104.725;
    public static double SH6_H1 = 150;

    // Shooter / intake tuning
    public static double SHOOTER_SPEED    = 1100;  // ticks/s
    public static double HOOD_POSITION    = 0.45;
    public static double SHOOT_DURATION_1 = 2.75;  // first shot seconds
    public static double SHOOT_DURATION   = 0.75;  // subsequent shots seconds

    // ── Internal ───────────────────────────────────────────────────────────────
    private static final Style SHOOT  = new Style("shoot",  "#FF4444", 1.2f);
    private static final Style INTAKE = new Style("intake", "#44DD44", 1.2f);
    private static final Style GATE   = new Style("gate",   "#4488FF", 1.2f);
    private static final Style START_STYLE = new Style("start", "#FFFFFF", 1.5f);

    private Follower follower;
    private PathChain[] paths;
    private int pathIndex = -1;

    private enum Phase { DRIVING, SHOOTING, DONE }
    private Phase  phase      = Phase.DONE;
    private double shootEndTime;
    private int    shootCount;
    private Intake     intake;
    private ServoGate  servoGate;
    private DcMotorEx  shooterMotor;
    private Servo      hood;

    private enum PathType { SHOOT, INTAKE, NEUTRAL }
    private static final PathType[] PATH_TYPES = {
        PathType.SHOOT, PathType.INTAKE, PathType.SHOOT, PathType.INTAKE,
        PathType.SHOOT, PathType.INTAKE, PathType.SHOOT, PathType.INTAKE,
        PathType.SHOOT, PathType.INTAKE, PathType.SHOOT
    };

    @Override
    public void init() {
        follower = Constants.PedroPathing.createFollower(hardwareMap);
        Drawing.init();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        intake       = new Intake(hardwareMap);
        servoGate    = new ServoGate(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(250, 0, 0, 15));
        hood = hardwareMap.get(Servo.class, "hood");
        servoGate.closeGate();
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
        phase = Phase.DRIVING; shootCount = 0;
    }

    @Override
    public void loop() {
        follower.update();

        if (phase == Phase.SHOOTING) {
            shooterMotor.setVelocity(SHOOTER_SPEED);
            hood.setPosition(HOOD_POSITION);
            servoGate.openGate();
            intake.setState(Intake.CaseModes.ON);
            intake.update();
            if (getRuntime() >= shootEndTime) {
                servoGate.closeGate();
                shooterMotor.setVelocity(0);
                intake.setState(Intake.CaseModes.OFF);
                intake.update();
                phase = Phase.DRIVING;
                advancePath();
            }

        } else if (phase == Phase.DRIVING && pathIndex >= 0) {
            PathType type = PATH_TYPES[pathIndex];
            if (type == PathType.SHOOT) {
                shooterMotor.setVelocity(SHOOTER_SPEED);
                hood.setPosition(HOOD_POSITION);
                intake.setState(Intake.CaseModes.OFF);
            } else if (type == PathType.INTAKE) {
                shooterMotor.setVelocity(0);
                intake.setState(Intake.CaseModes.ON);
            } else {
                shooterMotor.setVelocity(0);
                intake.setState(Intake.CaseModes.OFF);
            }
            intake.update();

            if (!follower.isBusy()) {
                if (type == PathType.SHOOT) {
                    double dur = (shootCount == 0) ? SHOOT_DURATION_1 : SHOOT_DURATION;
                    shootEndTime = getRuntime() + dur;
                    shootCount++;
                    phase = Phase.SHOOTING;
                } else {
                    servoGate.closeGate();
                    intake.setState(Intake.CaseModes.OFF);
                    intake.update();
                    advancePath();
                }
            }

        } else {
            // DONE
            shooterMotor.setVelocity(0);
            servoGate.closeGate();
            intake.setState(Intake.CaseModes.OFF);
            intake.update();
        }

        Drawing.drawDebug(follower);
        telemetry.addData("Status",   pathIndex < 0 ? "Done — Stop/Init to re-run" : "Path " + (pathIndex + 1) + " / " + paths.length);
        telemetry.addData("Phase",    phase);
        telemetry.addData("Flywheel", (int) shooterMotor.getVelocity() + " / " + (int) SHOOTER_SPEED + " tps");
        telemetry.update();
    }

    private void advancePath() {
        pathIndex++;
        if (pathIndex < paths.length) {
            follower.followPath(paths[pathIndex], true);
        } else {
            pathIndex = -1;
            phase      = Phase.DONE;
        }
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setVelocity(0);
        if (servoGate    != null) servoGate.closeGate();
        if (intake       != null) { intake.setState(Intake.CaseModes.OFF); intake.update(); }
    }

    /** Builds all PathChains from current static field values. */
    private PathChain[] buildPaths() {
        PathChain shoot1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SH1_X0, SH1_Y0), new Pose(SH1_X1, SH1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H0), Math.toRadians(SH1_H1)).build();

        PathChain spikeIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SH1_X1, SH1_Y1), new Pose(SP1_CX, SP1_CY), new Pose(SP1_X1, SP1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(180)).build();

        PathChain shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SP1_X1, SP1_Y1), new Pose(SH2_CX, SH2_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(SH1_H1)).build();

        PathChain gateIntake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(GI1_CX, GI1_CY), new Pose(GI1_X1, GI1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(160)).build();

        PathChain shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(GI1_X1, GI1_Y1), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(SH1_H1)).build();

        PathChain gateIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(GI2_CX, GI2_CY), new Pose(GI1_X1, GI1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(160)).build();

        PathChain shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(GI1_X1, GI1_Y1), new Pose(SH4_CX, SH4_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(SH1_H1)).build();

        PathChain spikeIntake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(SP2_CX, SP2_CY), new Pose(SP2_X1, SP2_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(180)).build();

        PathChain shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SP2_X1, SP2_Y1), new Pose(SH5_CX, SH5_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(SH1_H1)).build();

        PathChain spikeIntake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(SP3_CX, SP3_CY), new Pose(SP3_X1, SP3_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(180)).build();

        PathChain shoot6 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SP3_X1, SP3_Y1), new Pose(SH6_X1, SH6_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(SH6_H1)).build();

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
