package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.NewDriveSpeeds;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


// ALL SHOOTER SPEEDS ARE IN TICKS/SECOND. DO NOT, I REPEAT DO NOT, USE DEGREES/SECOND
@TeleOp(name = "TeleOp Blue", group = "Teleop")
public class TeleOpBlue extends AbstractOpMode {
    public static final double PROJECTILE_SPEED_CM = 5000; // Needs tuninggggggg
    RevBlinkinLedDriver.BlinkinPattern green;
    RevBlinkinLedDriver.BlinkinPattern red;
    double launchPower = 0;

    Location gatePosition = new Location(60, -163, -124);
    Location parkPosition = new Location(130.7, 65, 90);
    Pose2D goal = Constants.BLUE_CENTER_GOAL;
    double headingOffset = 0;


    public void setTargets() {
        limelight = new LimeLight(hardwareMap, 20);
    }

    @Override
    public void runOpMode() {
        startHardware();
        setTargets();
        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        red = RevBlinkinLedDriver.BlinkinPattern.RED;
        double velocityDeg = drivebase.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        double position = 0.5;

//        drivebase.setCurrentPose(0,0,-Math.PI/2);
        drivebase.setCurrentPose(PoseStorage.startPose);

        waitForStart();
        while (opModeIsActive()) {
//            drivebase.update();
            double kP = 0.02;
            double kD = 0.0015;
            LLResult aprilTag = drivebase.update(limelight);
            double distance = drivebase.distanceToGoal(drivebase.getPosition(), goal);
            double angleError = drivebase.angleToGoal(drivebase.getPosition(), goal);
            double joystick_y = gamepad1.left_stick_x; // Forward/backward
            double joystick_x = gamepad1.left_stick_y;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation
            double velocityX = drivebase.getOdo().getVelX(DistanceUnit.CM);
            double velocityY = drivebase.getOdo().getVelY(DistanceUnit.CM);
            double headingVelocity = drivebase.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            Pose2D currentPosition = drivebase.getPosition();

            if(gamepad1.left_bumper && gamepad1.a) {
                drivebase.setstate(NewDriveSpeeds.DRIVE_FULL);
            }
            else if(gamepad1.left_bumper && gamepad1.b) {
                drivebase.setstate(NewDriveSpeeds.DRIVE_ALMOST_FULL);
            }
            else if(gamepad1.left_bumper && gamepad1.x) {
                drivebase.setstate(NewDriveSpeeds.DRIVE_SEVENTY_PERCENT);
            }
            else if(gamepad1.left_bumper && gamepad1.y) {
                drivebase.setstate(NewDriveSpeeds.DRIVE_HALF);
            }
//            if (gamepad2.left_trigger_pressed) {
//                turret.setkP(turret.getkP() + 0.0001);
//                if (gamepad2.left_trigger_pressed) {
//                    if (gamepad2.right_trigger_pressed)
//                {
//                    turret.setkP(turret.getkP() - 0.0001);
//                    if (gamepad2.left_trigger_pressed)
//                    {
            if (gamepad1.dpadUpWasPressed()) {
                position += 0.1;

            } else if (gamepad1.dpadDownWasPressed()) {
                position -= 0.1;
            } else {
                shooter.setHoodPosition(position);
            }
            if (aprilTag != null) {
                drivebase.setCurrentPose(aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).x,
                        aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).y);
                telemetry.addData("BotPose", aprilTag.getBotpose_MT2().getPosition());
            }


            if (gamepad1.start) {
                drivebase.resetHeading(-90);
            }

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
            else if (gamepad1.left_trigger_pressed){
                // Shoot on the move code???? Maybe it'll workkkkk
                double vPerpendicular = velocityX * Math.cos(angleError) - velocityY * Math.sin(angleError);
                double leadAngle = Math.atan2(vPerpendicular, PROJECTILE_SPEED_CM);
                double correctedError = angleError + leadAngle;
                double errorDeg = correctedError * (180 / Math.PI);
                servoGate.openGate();
                joystick_rx = joystick_rx + errorDeg * kP - velocityDeg * kD;

                if (shooter.getShooterVelocity() >= shooter.distanceToSpeed(distance)) {
                    intake.setState(Intake.CaseModes.SIXTY_PERCENT_SPEED);
                    gamepad1.rumble(1000);
                }
            }
            else if (gamepad1.x || gamepad2.square || gamepad2.triangle) {
                shooter.setShooterSpeedNear(1100);
                servoGate.openGate();
                if (shooter.getShooterVelocity() >= 1100) {
                    intake.setState(Intake.CaseModes.ON);
                }
            }
            else {
                shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
            }

            if (gamepad1.triangle) {
                drivebase.driveToPosition(gatePosition, 0, telemetry);
                intake.setState(Intake.CaseModes.ON);
            } else if (gamepad1.share) {
                drivebase.driveToPosition(parkPosition, 0, telemetry);
            } else {
                drivebase.drive(joystick_y, joystick_x, joystick_rx, headingOffset);
            }

            telemetry.addData("Angle from goal", angleError * 180/Math.PI);
            telemetry.addData("Distance from goal", distance);
            telemetry.addData("Shooter Stuff: ", shooter.telemetryUpdate());
            telemetry.addData("LaunchPower", this.launchPower);
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.addData("Number of artifacts", intake.numberOfArtifacts());
            if (aprilTag != null) {
                telemetry.addData("AprilTag", aprilTag.getBotpose_MT2());
            }
            telemetry.update();
            intake.update();
            shooter.update();
            double turretErr = 0;
            if(aprilTag != null)
            {
                turretErr = aprilTag.getTx();
            }
            turret.update(turretErr);
            //turret.update(drive.angleToGoal(drive.getPosition(), goal));

        }
    }
}