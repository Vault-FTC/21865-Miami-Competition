package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class SpeedTuner extends LinearOpMode {
    public LimeLight Limelight;
    Intake intake;
    Lights light;
    double launchPower = 0;
    Pose2D goal = Constants.BLUE_CENTER_GOAL;
    Pose2D startPose;
    double headingOffset = 0;


    public void setTargets() {
        Limelight = new LimeLight(hardwareMap, 20);
    }

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        light = new Lights(hardwareMap);
        Drivebase drive = new Drivebase(hardwareMap);
        ServoGate servoGate = new ServoGate(hardwareMap);
        Shooter launcher = new Shooter(hardwareMap);
        setTargets();

        double position = 0.5;
        double shooterSpeed = 0;

        drive.setCurrentPose(PoseStorage.startPose);

        waitForStart();
        while (opModeIsActive()) {
            LLResult aprilTag = drive.update(Limelight);
            double distance = drive.distanceToGoal(drive.getPosition(), goal);
            double angleError = drive.angleToGoal(drive.getPosition(), goal);
            boolean autoShoot = gamepad1.right_bumper;
            double joystick_y = gamepad1.left_stick_x; // Forward/backward
            double joystick_x = gamepad1.left_stick_y;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation


            if (gamepad1.dpadUpWasPressed()) {
                shooterSpeed += 10;
            } else if (gamepad1.dpadDownWasPressed()) {
                shooterSpeed -= 10;
            } else if (gamepad1.dpadLeftWasPressed()) {
                shooterSpeed = 0;
            } else if (gamepad1.dpadRightWasPressed()) {
                shooterSpeed = 1500;
            }

//            if (aprilTag != null) {
//                drive.setCurrentPose(aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).x,
//                        aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).y);
//            }


            if (gamepad1.start) {
                drive.resetHeading(-90);
            }
            if (gamepad1.x) {
                launcher.setShooterSpeedNear(1100);
                servoGate.openGate();
                if (launcher.getShooterVelocity() >= 1100) {
                    intake.spinIntake(0.95);
                }
            } else if (gamepad1.left_bumper) {
                intake.spinIntake(0.95);

            } else if (gamepad1.b) {
                intake.spinIntake(-0.95);
                launcher.setShooterSpeedNear(-900);
            } else if (autoShoot) {
//              joystick_rx = joystick_rx - aprilTag.getTargetXDegrees() * 0.02;
                joystick_rx = joystick_rx + angleError * ((180/Math.PI) * 0.02);
                servoGate.openGate();
                gamepad1.rumble(1000);
                if (distance < 240) {
                    launcher.setShooterSpeedNear(launcher.distanceToSpeed(distance));
                } else {
                    launcher.setShooterSpeedFar(launcher.distanceToSpeed(distance));
                }

                launcher.setHoodPosition(launcher.distanceToHoodPosition(distance));
                if (launcher.getShooterVelocity() >= launcher.distanceToSpeed(distance)) {
                    intake.spinIntake(0.95);
                }
            } else {
                launcher.setShooterSpeedNear(shooterSpeed);
                intake.spinIntake(0);
                servoGate.closeGate();
            }



            drive.drive(joystick_y, joystick_x, joystick_rx, headingOffset);

//            telemetry.addData("Angle from goal", angleError * 180/Math.PI);
            telemetry.addData("Distance from goal", distance);
//            telemetry.addData("Shooter Stuff: ", launcher.telemetryUpdate());
            telemetry.addData("Target ShooterSpeed", shooterSpeed);
            telemetry.addData("LaunchPower", this.launchPower);
            telemetry.addData("Position", drive.getPositionTelemetry());
            if (aprilTag != null) {
                telemetry.addData("AprilTag", aprilTag.getBotpose_MT2());
                telemetry.addData("AprilTag distance from goal", Math.hypot(aprilTag.getBotpose_MT2().getPosition().x - Constants.BLUE_CENTER_GOAL.getX(DistanceUnit.CM), aprilTag.getBotpose_MT2().getPosition().y) - Constants.BLUE_CENTER_GOAL.getY(DistanceUnit.CM));
            }
            telemetry.update();
        }
    }
}
