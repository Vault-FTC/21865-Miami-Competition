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
 * Paste final values back into FarPathsRed.java.
 * All coordinates are in Red-alliance space (already mirrored — no further flip needed).
 */
@Configurable
@TeleOp(name = "TUNE: Far - Red", group = "TUNING - delete when done")
public class TuneFarRed extends OpMode {

    // ── Paste final values back into FarPathsRed.java ────────────────────────

    public static double START_X = 90,   START_Y = 6,   START_H = 90;

    // Shoot1 (Red space)
    public static double SH1_X1 = 87,   SH1_Y1 = 12;
    public static double SH1_H1 = 75;

    // Intake1 (Red space)
    public static double IN1_CX = 82.720, IN1_CY = 33;
    public static double IN1_X1 = 124,    IN1_Y1 = 50;

    // Shoot2 (Red space)
    public static double SH2_H1 = 51;

    // Intake2 (Red space)
    public static double IN2_X1 = 132,  IN2_Y1 = 24;

    // Shoot3 (Red space)
    public static double SH3_X0 = 132,    SH3_Y0 = 24;
    public static double SH3_X1 = 87,     SH3_Y1 = 20;
    public static double SH3_H1 = 51;

    // Intake3 (Red space)
    public static double IN3_X0 = 87,     IN3_Y0 = 11.971;
    public static double IN3_X1 = 132,    IN3_Y1 = 30;

    // Shoot4 (Red space)
    public static double SH4_X0 = 134,    SH4_Y0 = 23.627;

    // Park (Red space)
    public static double PARK_X = 87,     PARK_Y = 31.364;
    public static double PARK_H = 90;

    // Shooter / intake tuning — match FarAuto.java TimedShootCommand values
    public static double SHOOTER_SPEED    = 1400;  // ticks/s for shot 1; auton uses 1450 for shots 2–4
    public static double HOOD_POSITION    = 0.70;  // auton uses 0.7 for all shots
    public static double SHOOT_DURATION_1 = 3.0;   // shot 1 (auton: 3.0s)
    public static double SHOOT_DURATION   = 1.0;   // shots 2–4 (auton: 1.0s)

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
        PathType.SHOOT, PathType.INTAKE, PathType.SHOOT, PathType.NEUTRAL
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
                new BezierLine(new Pose(START_X, START_Y), new Pose(SH1_X1, SH1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(START_H), Math.toRadians(SH1_H1)).build();

        PathChain intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(SH1_X1, SH1_Y1), new Pose(IN1_CX, IN1_CY), new Pose(IN1_X1, IN1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH1_H1), Math.toRadians(0)).build();

        PathChain shoot2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(IN1_X1, IN1_Y1), new Pose(SH1_X1, SH1_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(SH2_H1)).build();

        PathChain intake2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SH1_X1, SH1_Y1), new Pose(IN2_X1, IN2_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH2_H1), Math.toRadians(0)).build();

        PathChain shoot3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(IN2_X1, IN2_Y1), new Pose(SH3_X1, SH3_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(SH3_H1)).build();

        PathChain intake3 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(SH3_X1, SH3_Y1), new Pose(IN3_X1, IN3_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(SH3_H1), Math.toRadians(0)).build();

        PathChain shoot4 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(IN3_X1, IN3_Y1), new Pose(SH3_X1, SH3_Y1))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(SH3_H1)).build();

        PathChain park = follower.pathBuilder().addPath(
                new BezierLine(new Pose(IN3_X1, IN3_Y1), new Pose(PARK_X, PARK_Y))
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
