package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.NewDriveSpeeds;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


// ALL SHOOTER SPEEDS ARE IN TICKS/SECOND. DO NOT, I REPEAT DO NOT, USE DEGREES/SECOND
@TeleOp(name = "TeleOp Blue", group = "Teleop")
public class TeleOpBlue extends AbstractOpMode {
    public static final double PROJECTILE_SPEED_CM = 5000; // Needs tuninggggggg
    RevBlinkinLedDriver.BlinkinPattern green;
    RevBlinkinLedDriver.BlinkinPattern red;
    double launchPower = 0;
    Location gatePosition = new Location(21.1, 158, 150);
    Location parkPosition = new Location(130.7, 65, 90);
    Pose2D goal = Constants.BLUE_CENTER_GOAL;
    double headingOffset = -Math.PI/2;

    /** Degrees of aim trim added per button press. */
    private static final double AIM_TRIM_STEP_DEG = 1.0;

    public void setTargets() {
        limelight = new LimeLight(hardwareMap, 20, drivebase);
    }

    @Override
    public void runOpMode() {
        startHardware();
        setTargets();
        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        red = RevBlinkinLedDriver.BlinkinPattern.RED;
        waitForStart();

        // Restore X/Y from the auto ending pose.
        // DO NOT touch the heading: the Pinpoint IMU hardware tracks heading continuously
        // and already holds the correct value from the end of auto.  Pedro's
        // follower.getPose().getHeading() lives in a different coordinate frame and will
        // produce the wrong Pinpoint heading if used here.
        drivebase.setCurrentPose(
                PoseStorage.startPose.getX() * 2.54,
                PoseStorage.startPose.getY() * 2.54
        );

        while (opModeIsActive()) {
            double distance = drivebase.distanceToGoal(drivebase.getPosition(), goal);
            double angleError = drivebase.angleToGoal(drivebase.getPosition(), goal);
            double joystick_y = gamepad1.left_stick_x; // Forward/backward
            double joystick_x = gamepad1.left_stick_y;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation

//            if(gamepad1.left_bumper && gamepad1.a)
//            {
//                drivebase.setstate(NewDriveSpeeds.DRIVE_FULL);
//            }
//            else if(gamepad1.left_bumper && gamepad1.b)
//            {
//                drivebase.setstate(NewDriveSpeeds.DRIVE_ALMOST_FULL);
//            }
//            else if(gamepad1.left_bumper && gamepad1.x)
//            {
//                drivebase.setstate(NewDriveSpeeds.DRIVE_SEVENTY_PERCENT);
//            }
//            else if(gamepad1.left_bumper && gamepad1.y)
//            {
//                drivebase.setstate(NewDriveSpeeds.DRIVE_HALF);
//            }

            if (gamepad1.startWasPressed()) {
                drivebase.resetHeading();
            }

            // ── Gamepad 2: manual aim trim ──────────────────────────────────────
            // Press LB → aim 1° left,  Press RB → aim 1° right,  Start → reset to 0.
            if (gamepad2.leftBumperWasPressed()) {
                shooter.adjustAimTrim(-AIM_TRIM_STEP_DEG);
            } else if (gamepad2.rightBumperWasPressed()) {
                shooter.adjustAimTrim(AIM_TRIM_STEP_DEG);
            }
            if (gamepad2.startWasPressed()) {
                shooter.resetAimTrim();
            }
            // ───────────────────────────────────────────────────────────────────
            if (gamepad1.left_bumper) {
                intake.setState(Intake.CaseModes.ON);
            } else if (gamepad1.b || gamepad1.circle) {
                intake.setState(Intake.CaseModes.REVERSE);
                shooter.setState(Shooter.CaseModes.REVERSE);
            } else if (gamepad1.right_bumper) {
                shooter.setState(Shooter.CaseModes.SHOOT_NEAR);
            } else if (gamepad1.right_trigger_pressed)  {
                shooter.setState(Shooter.CaseModes.SHOOT_FAR);
            }
            else if (gamepad1.left_trigger_pressed) {
                // Shoot on the move: lead-angle compensated aim, distance-based speed/hood.
                shooter.setState(Shooter.CaseModes.SHOOT_ON_MOVE);
            } else if (gamepad1.share) {
                shooter.setState(Shooter.CaseModes.SHOOT_LIMELIGHT_AIM);
            }

            else if (gamepad1.square || gamepad2.square || gamepad2.triangle) {
                shooter.setState(Shooter.CaseModes.SHOOT_NO_AIM);
            } else {
                shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
            }

            if (gamepad1.triangle) {
                drivebase.driveToPosition(gatePosition, 0, telemetry);
                intake.setState(Intake.CaseModes.ON);
            }
//            } else if (gamepad1.share) {
//                drivebase.driveToPosition(parkPosition, 0, telemetry);
//            }
            else {
                drivebase.drive(joystick_y, joystick_x, joystick_rx, headingOffset);
            }

            telemetry.addData("Angle from goal", angleError * 180/Math.PI);
            telemetry.addData("Goal dir (deg)", Math.toDegrees(Math.atan2(
                    goal.getY(DistanceUnit.CM) - drivebase.getPosition().getY(DistanceUnit.CM),
                    goal.getX(DistanceUnit.CM) - drivebase.getPosition().getX(DistanceUnit.CM))));
            telemetry.addData("Distance from goal", distance);
            telemetry.addData("Shooter Stuff: ", shooter.telemetryUpdate());
            telemetry.addData("LaunchPower", this.launchPower);
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.addData("Has Three Artifacts", intake.hasThreeArtifacts());
            telemetry.addData("Intake Current Draw", intake.currentDraw());
            limelight.update();
            limelight.addTelemetry(telemetry);
            telemetry.update();
            intake.update();
            shooter.update();
            drivebase.update();
        }
    }
}