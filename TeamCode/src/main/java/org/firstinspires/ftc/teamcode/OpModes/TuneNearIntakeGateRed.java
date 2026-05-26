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
 * Paste final values back into NearIntakeGatePathsRed.java.
 * All coordinates are in Red-alliance space (already mirrored — no further flip needed).
 */
@Configurable
@TeleOp(name = "TUNE: Near Intake Gate - Red", group = "TUNING - delete when done")
public class TuneNearIntakeGateRed extends OpMode {

    // ── Paste final values back into NearIntakeGatePathsRed.java ─────────────

    public static double START_X = 124,  START_Y = 127,  START_H = 40;
    public static double SHOOT_X = 99,   SHOOT_Y = 95;

    // Intake1 (Red space)
    public static double IN1_CX = 81,    IN1_CY = 54;
    public static double IN1_X1 = 129,   IN1_Y1 = 63;

    // Shoot2 (Red space)
    public static double SH2_CX = 99,    SH2_CY = 70;
    public static double SH2_H1 = 42;

    // Intake2 (Red space)
    public static double IN2_CX = 74,  IN2_CY = 70;
    public static double IN2_X1 = 135, IN2_Y1 = 64.5;
    public static double IN2_H1 = 20;

    // Shoot3/4/5 (Red space)
    public static double SH345_CX = 81, SH345_CY = 80;
    public static double SH345_H0 = 20, SH345_H1 = 42;

    // Intake3/4 (Red space)
    public static double IN34_CX = 81,  IN34_CY = 54;
    public static double IN34_X1 = 135, IN34_Y1 = 64.5;
    public static double IN34_H0 = 42,  IN34_H1 = 20;

    // Intake5 (Red space)
    public static double IN5_CX = 76,  IN5_CY = 83;
    public static double IN5_X1 = 126, IN5_Y1 = 86;

    // Shoot6 (Red space)
    public static double SH6_X1 = 89,    SH6_Y1 = 105;
    public static double SH6_H1 = 34;

    // Park (Red space)
    public static double PARK_X = 109,   PARK_Y = 90;
    public static double PARK_H = 270;

    // Shooter / intake tuning
    public static double SHOOTER_SPEED    = 1100;  // ticks/s
    public static double HOOD_POSITION    = 0.45;
    public static double SHOOT_DURATION_1 = 2.75;  // first shot seconds
    public static double SHOOT_DURATION   = 0.75;  // subsequent shots seconds

    // ── Internal ───────────────────────────────────────────────────────────────
    private static final Style SHOOT  = new Style("shoot",  "#FF4444", 1.2f);
    private static final Style INTAKE = new Style("intake", "#44DD44", 1.2f);
    private static final Style PARK   = new Style("park",   "#FFFF44", 1.2f);
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
                new BezierLine(new Pose(START_X, START_Y), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(START_H), Math.toRadians(45)).build();

        PathChain intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(IN1_CX, IN1_CY), new Pose(IN1_X1, IN1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)).build();

        PathChain shoot2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(IN1_X1, IN1_Y1), new Pose(SH2_CX, SH2_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(SH2_H1)).build();

        PathChain intake2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(IN2_CX, IN2_CY), new Pose(IN2_X1, IN2_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH2_H1), Math.toRadians(IN2_H1)).build();

        PathChain shoot3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(IN2_X1, IN2_Y1), new Pose(SH345_CX, SH345_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(SH345_H0), Math.toRadians(SH345_H1)).build();

        PathChain intake3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(IN34_CX, IN34_CY), new Pose(IN34_X1, IN34_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(IN34_H0), Math.toRadians(IN34_H1)).build();

        PathChain shoot4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(IN34_X1, IN34_Y1), new Pose(SH345_CX, SH345_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(SH345_H0), Math.toRadians(SH345_H1)).build();

        PathChain intake4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(IN34_CX, IN34_CY), new Pose(IN34_X1, IN34_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(IN34_H0), Math.toRadians(IN34_H1)).build();

        PathChain shoot5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(IN34_X1, IN34_Y1), new Pose(SH345_CX, SH345_CY), new Pose(SHOOT_X, SHOOT_Y))
        ).setLinearHeadingInterpolation(Math.toRadians(SH345_H0), Math.toRadians(SH345_H1)).build();

        PathChain intake5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SHOOT_X, SHOOT_Y), new Pose(IN5_CX, IN5_CY), new Pose(IN5_X1, IN5_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH345_H1), Math.toRadians(0)).build();

        PathChain shoot6 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(IN5_X1, IN5_Y1), new Pose(SH6_X1, SH6_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(SH6_H1)).build();

        return new PathChain[]{
                shoot1, intake1, shoot2, intake2, shoot3, intake3,
                shoot4, intake4, shoot5, intake5, shoot6
        };
    }

    /** Builds paths and draws them for preview during init_loop. */
    private void buildAndDrawPreview() {
        PathChain[] preview = buildPaths();
        Drawing.drawRobot(new Pose(START_X, START_Y, Math.toRadians(START_H)), START_STYLE);
        Drawing.drawPath(preview[0],  SHOOT);   // shoot1
        Drawing.drawPath(preview[1],  INTAKE);  // intake1
        Drawing.drawPath(preview[2],  SHOOT);   // shoot2
        Drawing.drawPath(preview[3],  INTAKE);  // intake2
        Drawing.drawPath(preview[4],  SHOOT);   // shoot3
        Drawing.drawPath(preview[5],  INTAKE);  // intake3
        Drawing.drawPath(preview[6],  SHOOT);   // shoot4
        Drawing.drawPath(preview[7],  INTAKE);  // intake4
        Drawing.drawPath(preview[8],  SHOOT);   // shoot5
        Drawing.drawPath(preview[9],  INTAKE);  // intake5
        Drawing.drawPath(preview[10], SHOOT);   // shoot6
        Drawing.sendPacket();
    }
}
