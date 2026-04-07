package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

@TeleOp
public class SpeedTuner extends AbstractOpMode {
    double launchPower = 0;
    Pose2D goal = Constants.BLUE_CENTER_GOAL;
    double headingOffset = 0;

    public void setTargets() {
        limelight = new LimeLight(hardwareMap, 20);
    }

    @Override
    public void runOpMode() {
        startHardware();
        setTargets();

        double position = 0.5;
        double shooterSpeed = 0;

        drivebase.setCurrentPose(PoseStorage.startPose);

        waitForStart();
        while (opModeIsActive()) {
            LLResult aprilTag = drivebase.update(limelight);
            double distance = drivebase.distanceToGoal(drivebase.getPosition(), goal);
            double angleError = drivebase.angleToGoal(drivebase.getPosition(), goal);
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
            } else if (gamepad1.triangleWasPressed()) {
                position += 0.1;
            } else if (gamepad1.crossWasPressed()){
                position -= 0.1;
            } else {
                shooter.setHoodPosition(position);
            }

//            if (aprilTag != null) {
//                drivebase.setCurrentPose(aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).x,
//                        aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).y);
//            }


            if (gamepad1.start) {
                drivebase.resetHeading(-90);
            }
            if (gamepad1.square) {
                servoGate.openGate();
                intake.setState(Intake.CaseModes.ON);
            } else if (gamepad1.left_bumper) {
                intake.setState(Intake.CaseModes.ON);

            } else if (gamepad1.b) {
                intake.setState(Intake.CaseModes.REVERSE);
                shooter.setShooterSpeedNear(-900);
            } else if (autoShoot) {
//              joystick_rx = joystick_rx - aprilTag.getTargetXDegrees() * 0.02;
                joystick_rx = joystick_rx + angleError * ((180/Math.PI) * 0.02);
//                servoGate.openGate();
//                gamepad1.rumble(1000);
//                if (distance < 240) {
//                    shooter.setShooterSpeedNear(shooter.distanceToSpeed(distance));
//                } else {
//                    shooter.setShooterSpeedFar(shooter.distanceToSpeed(distance));
//                }
//
//                shooter.setHoodPosition(shooter.distanceToHoodPosition(distance));
//                if (shooter.getShooterVelocity() >= shooter.distanceToSpeed(distance)) {
//                    intake.spinIntake(0.95);
//                }
            } else {
                shooter.setShooterSpeedNear(shooterSpeed);
                intake.setState(Intake.CaseModes.OFF);
                servoGate.closeGate();
            }



            drivebase.drive(joystick_y, joystick_x, joystick_rx, headingOffset);

//            telemetry.addData("Angle from goal", angleError * 180/Math.PI);
            telemetry.addData("Distance from goal", distance);
//            telemetry.addData("Shooter Stuff: ", shooter.telemetryUpdate());
            telemetry.addData("Target ShooterSpeed", shooterSpeed);
            telemetry.addData("LaunchPower", this.launchPower);
            telemetry.addData("Hood Position", position);
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            if (aprilTag != null) {
                telemetry.addData("AprilTag", aprilTag.getBotpose_MT2());
                telemetry.addData("AprilTag distance from goal", Math.hypot(aprilTag.getBotpose_MT2().getPosition().x - Constants.BLUE_CENTER_GOAL.getX(DistanceUnit.CM), aprilTag.getBotpose_MT2().getPosition().y) - Constants.BLUE_CENTER_GOAL.getY(DistanceUnit.CM));
            }
            telemetry.update();
        }
    }
}
