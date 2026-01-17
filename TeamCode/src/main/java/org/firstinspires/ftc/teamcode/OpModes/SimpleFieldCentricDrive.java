package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.Autonomous.Location;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;


// ALL SHOOTER SPEEDS ARE IN TICKS/SECOND. DO NOT, I REPEAT DO NOT, USE DEGREES/SECOND
@TeleOp(name = "TeleOp Blue", group = "Teleop")
public class SimpleFieldCentricDrive extends LinearOpMode {

    public LimeLight Limelight;
    Intake intake;
    RevBlinkinLedDriver.BlinkinPattern green;
    RevBlinkinLedDriver.BlinkinPattern red;
    Lights light;
    double launchPower = 0;
    Pose2D goal = FieldConstants.BLUE_CENTER_GOAL;



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
        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        red = RevBlinkinLedDriver.BlinkinPattern.RED;

        double position = 0.5;

        drive.setCurrentPose(0,0,-Math.PI/2);

        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            LLResultTypes.FiducialResult aprilTag = drive.update(Limelight);
            double distance = drive.distanceToGoal(drive.getPosition(), goal);
            double angleError = drive.angleToGoal(drive.getPosition(), goal);
            boolean autoShoot = gamepad1.right_bumper;
            double joystick_y = gamepad1.left_stick_x; // Forward/backward
            double joystick_x = gamepad1.left_stick_y;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation

            if (gamepad1.dpadUpWasPressed()) {
                position += 0.1;

            } else if (gamepad1.dpadDownWasPressed()) {
                position -= 0.1;
            } else {
                launcher.setHoodPosition(position);
            }
//            if (aprilTag != null) {
//                drive.setCurrentPose(aprilTag.getRobotPoseFieldSpace().getPosition().y, aprilTag.getRobotPoseFieldSpace().getPosition().x, aprilTag.getRobotPoseFieldSpace().getOrientation().getYaw() + Math.PI);
//            }


            if (gamepad1.start) {
                drive.resetHeading(0);
            }
            if (gamepad1.x) {
               intake.spinIntake(0.95);
               servoGate.openGate();
            } else if (gamepad1.left_bumper) {
                intake.spinIntake(0.95);

            } else if (gamepad1.b) {
                intake.spinIntake(-0.95);
                launcher.setShooterSpeed(-900);
            } else {
               intake.spinIntake(0);
               launcher.setShooterSpeed(0);
               servoGate.closeGate();
            }

            if (gamepad1.dpad_left && aprilTag != null) {
                joystick_rx = joystick_rx - aprilTag.getTargetXDegrees() * 0.02;
                launcher.setShooterSpeed(launcher.distanceToSpeed(distance));
                launcher.setHoodPosition(launcher.distanceToHoodPosition(distance));
            }

            if (autoShoot) {
//              joystick_rx = joystick_rx - aprilTag.getTargetXDegrees() * 0.02;
                joystick_rx = joystick_rx + angleError * ((180/Math.PI) * 0.02);
                servoGate.openGate();
                gamepad1.rumble(1000);
                launcher.setShooterSpeed(launcher.distanceToSpeed(distance));
                launcher.setHoodPosition(launcher.distanceToHoodPosition(distance));
            } else {
                launcher.setShooterSpeed(0);
            }

            drive.drive(joystick_y, joystick_x, joystick_rx);
            if (Limelight.getResult() != null) {
                telemetry.addData("Limelight Field X", Limelight.getFieldPose().getPosition().x);
                telemetry.addData("Limelight Field Y", Limelight.getFieldPose().getPosition().y);
                telemetry.addData("Limelight Field Heading", Limelight.getFieldPose().getOrientation().getYaw());
            }
            telemetry.addData("Angle from goal", angleError * 180/Math.PI);
            telemetry.addData("Distance from goal", distance);
            telemetry.addData("Shooter Stuff: ", launcher.telemetryUpdate());
            telemetry.addData("LaunchPower", this.launchPower);
            telemetry.addData("Position", drive.getPositionTelemetry());
            if (aprilTag != null) {
                telemetry.addData("AprilTag", aprilTag.getTargetXDegrees());
            }
            telemetry.update();
        }
    }
}