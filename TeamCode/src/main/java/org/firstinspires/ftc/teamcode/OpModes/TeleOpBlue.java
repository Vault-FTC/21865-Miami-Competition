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
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;


// ALL SHOOTER SPEEDS ARE IN TICKS/SECOND. DO NOT, I REPEAT DO NOT, USE DEGREES/SECOND
@TeleOp(name = "TeleOp Blue", group = "Teleop")
public class TeleOpBlue extends LinearOpMode {
    public static final double PROJECTILE_SPEED_CM = 5000; // Needs tuninggggggg

    public LimeLight limeLight;
    Intake intake;
    RevBlinkinLedDriver.BlinkinPattern green;
    RevBlinkinLedDriver.BlinkinPattern red;
    Drivebase drivebase;
    Lights lights;
    double launchPower = 0;

    Location gatePosition = new Location(60, -163, -124);
    Pose2D goal = Constants.BLUE_CENTER_GOAL;
    double headingOffset = 0;


    public void setTargets() {
        limeLight = new LimeLight(hardwareMap, 20);
    }

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        lights = new Lights(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        Drivebase drive = new Drivebase(hardwareMap);
        ServoGate servoGate = new ServoGate(hardwareMap);
        Shooter launcher = new Shooter(hardwareMap);
        setTargets();
        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        red = RevBlinkinLedDriver.BlinkinPattern.RED;
        double velocityDeg = drive.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        double position = 0.5;

//        drive.setCurrentPose(0,0,-Math.PI/2);
        drive.setCurrentPose(PoseStorage.startPose);

        waitForStart();
        while (opModeIsActive()) {
//            drive.update();
            double kP = 0.02;
            double kD = 0.0015;
            LLResult aprilTag = drive.update(limeLight);
            double distance = drive.distanceToGoal(drive.getPosition(), goal);
            double angleError = drive.angleToGoal(drive.getPosition(), goal);
            boolean autoShoot = gamepad1.right_bumper;
            double joystick_y = gamepad1.left_stick_x; // Forward/backward
            double joystick_x = gamepad1.left_stick_y;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation
            double velocityX = drivebase.getOdo().getVelX(DistanceUnit.CM);
            double velocityY = drivebase.getOdo().getVelY(DistanceUnit.CM);
            double headingVelocity = drivebase.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            Pose2D currentPosition = drivebase.getPosition();


            if (gamepad1.dpadUpWasPressed()) {
                position += 0.1;

            } else if (gamepad1.dpadDownWasPressed()) {
                position -= 0.1;
            } else {
                launcher.setHoodPosition(position);
            }
            if (aprilTag != null) {
                drive.setCurrentPose(aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).x,
                        aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).y);
                telemetry.addData("BotPose", aprilTag.getBotpose_MT2().getPosition());
            }


            if (gamepad1.start) {
                drive.resetHeading(-90);
            }

            if (gamepad1.left_bumper) {
                intake.spinIntake(1);

            } else if (gamepad1.b) {
                intake.spinIntake(-0.95);
                launcher.setShooterSpeedNear(-900);
            } else if (autoShoot) {

                double errorDeg = angleError * (180 / Math.PI);
                joystick_rx = joystick_rx + errorDeg * kP - velocityDeg * kD;
                servoGate.openGate();
                if (Math.abs(angleError * ((180/Math.PI))) < 2.5 && Math.abs(velocityDeg) < 10) {
                    intake.spinIntake(1);
                    gamepad1.rumble(1000);
                }
            } else if (gamepad1.right_trigger_pressed)  {
                double errorDeg = (angleError+0.1) * (180 / Math.PI);
                joystick_rx = joystick_rx + errorDeg * kP - velocityDeg * kD;
                servoGate.openGate();
                if (Math.abs((angleError+0.1) * ((180/Math.PI))) < 1 && launcher.getShooterVelocity() >= launcher.distanceToSpeed(distance)) {
                    intake.spinIntake(0.6);
                    gamepad1.rumble(1000);
                }
            } else if (gamepad1.left_trigger_pressed){
                // Shoot on the move code???? Maybe it'll workkkkk
                double vPerpendicular = velocityX * Math.cos(angleError) - velocityY * Math.sin(angleError);
                double leadAngle = Math.atan2(vPerpendicular, PROJECTILE_SPEED_CM);
                double correctedError = angleError + leadAngle;
                double errorDeg = correctedError * (180 / Math.PI);
                servoGate.openGate();
                joystick_rx = joystick_rx + errorDeg * kP - velocityDeg * kD;

                if (launcher.getShooterVelocity() >= launcher.distanceToSpeed(distance)) {
                    intake.spinIntake(0.6);
                    gamepad1.rumble(1000);
                }

                joystick_rx = joystick_rx + errorDeg * kP - velocityDeg * kD;
            } else if (gamepad1.x) {
                launcher.setShooterSpeedNear(1100);
                servoGate.openGate();
                if (launcher.getShooterVelocity() >= 1100) {
                    intake.spinIntake(0.95);
                }
            } else if (distance < 240) {
                launcher.setShooterSpeedNear(launcher.distanceToSpeed(distance));
                intake.spinIntake(0);
                servoGate.closeGate();
            } else {
                launcher.setShooterSpeedFar(launcher.distanceToSpeed(distance));
                intake.spinIntake(0);
                servoGate.closeGate();
            }

            launcher.setHoodPosition(launcher.distanceToHoodPosition(distance));
            if (gamepad1.triangle) {
                drive.driveToPosition(gatePosition, 0, telemetry);
                intake.spinIntake(0.9);
            }
            else {
                drive.drive(joystick_y, joystick_x, joystick_rx, headingOffset);
            }

            telemetry.addData("Angle from goal", angleError * 180/Math.PI);
            telemetry.addData("Distance from goal", distance);
            telemetry.addData("Shooter Stuff: ", launcher.telemetryUpdate());
            telemetry.addData("LaunchPower", this.launchPower);
            telemetry.addData("Position", drive.getPositionTelemetry());
            telemetry.addData("Number of artifacts", intake.numberOfArtifacts());
            if (aprilTag != null) {
                telemetry.addData("AprilTag", aprilTag.getBotpose_MT2());
            }
            telemetry.update();
        }
    }
}