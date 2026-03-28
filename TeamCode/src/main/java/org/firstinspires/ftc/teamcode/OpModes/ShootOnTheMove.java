//package org.firstinspires.ftc.teamcode.OpModes;
//
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.teamcode.Autonomous.Location;
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Lights;
//import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
//import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
//import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//@TeleOp
//public class ShootOnTheMove extends LinearOpMode {
//    public LimeLight limeLight;
//    Intake intake;
//    RevBlinkinLedDriver.BlinkinPattern green;
//    RevBlinkinLedDriver.BlinkinPattern red;
//    Drivebase drivebase;
//    Lights lights;
//    double launchPower = 0;
//    double error;
//    double targetHeading;
//
//    Location gatePosition = new Location(60, -163, -124);
//    Pose2D goal = Constants.BLUE_CENTER_GOAL;
//    double headingOffset = 0;
//
//
//    public void setTargets() {
//        limeLight = new LimeLight(hardwareMap, 20);
//    }
//
//    @Override
//    public void runOpMode() {
//        intake = new Intake(hardwareMap);
//        lights = new Lights(hardwareMap);
//        drivebase = new Drivebase(hardwareMap);
//        ServoGate servoGate = new ServoGate(hardwareMap);
//        Shooter launcher = new Shooter(hardwareMap);
//        setTargets();
//        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
//        red = RevBlinkinLedDriver.BlinkinPattern.RED;
//
//        drivebase.setCurrentPose(PoseStorage.startPose);
//
//        waitForStart();
//        while (opModeIsActive()) {
//            double velocityDeg = drivebase.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
////            drivebase.update();
//            double kP = 0.002;
//            LLResult aprilTag = drivebase.update(limeLight);
//            double distance = drivebase.distanceToGoal(drivebase.getPosition(), goal);
//            boolean autoShoot = gamepad1.right_bumper;
//            double joystick_y = gamepad1.left_stick_x; // Forward/backward
//            double joystick_x = gamepad1.left_stick_y;  // Strafe left/right
//            double joystick_rx = -gamepad1.right_stick_x; // Rotation
//
//            if (aprilTag != null) {
//                drivebase.setCurrentPose(aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).x,
//                        aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).y);
//                telemetry.addData("BotPose", aprilTag.getBotpose_MT2().getPosition());
//            }
//
//
//            if (gamepad1.start) {
//                drivebase.resetHeading(-90);
//            }
//
//            if (gamepad1.left_bumper) {
//                intake.spinIntake(1);
//
//            } else if (gamepad1.b) {
//                intake.spinIntake(-0.95);
//                launcher.setShooterSpeedNear(-900);
//            } else if (autoShoot) {
//                targetHeading = launcher.updateShootOnMove(this.drivebase.getOdo(), goal.getX(DistanceUnit.CM), goal.getY(DistanceUnit.CM));
//                error = targetHeading - drivebase.getOdo().getHeading(AngleUnit.DEGREES);
//                while (error > 180) error -= 360;
//                while (error < -180) error += 360;
//                joystick_rx = joystick_rx + error * kP;
//                servoGate.openGate();
//                if (Math.abs(error) < 2.5 && Math.abs(velocityDeg) < 10) {
//                    intake.spinIntake(1);
//                    gamepad1.rumble(1000);
//                }
//            } else if (gamepad1.x) {
//                launcher.setShooterSpeedNear(1100);
//                servoGate.openGate();
//                if (launcher.getShooterVelocity() >= 1100) {
//                    intake.spinIntake(0.95);
//                }
//            } else if (gamepad2.square) {
//                launcher.setShooterSpeedNear(1100);
//                servoGate.openGate();
//                if (launcher.getShooterVelocity() >= 1100) {
//                    intake.spinIntake(0.5);
//                }
//            } else if (gamepad2.triangle) {
//                launcher.setShooterSpeedNear(1100);
//                servoGate.openGate();
//            } else {
//                launcher.setShooterSpeedNear(launcher.distanceToSpeed(distance));
//                launcher.setHoodPosition(launcher.distanceToHoodPosition(distance));
//                intake.spinIntake(0);
//                servoGate.closeGate();
//            }
//
//
//            if (gamepad1.triangle) {
//                drivebase.driveToPosition(gatePosition, 0, telemetry);
//                intake.spinIntake(0.9);
//            }
//            else {
//                drivebase.drive(joystick_y, joystick_x, joystick_rx, headingOffset);
//            }
//
//            telemetry.addData("targetHeading", targetHeading);
//            telemetry.addData("currentHeading", drivebase.getOdo().getHeading(AngleUnit.DEGREES));
//            telemetry.addData("error", error);
//            telemetry.addData("joystick_rx contribution", error * kP);
//            telemetry.addData("Distance from goal", distance);
//            telemetry.addData("Shooter Stuff: ", launcher.telemetryUpdate());
//            telemetry.addData("LaunchPower", this.launchPower);
//            telemetry.addData("Position", drivebase.getPositionTelemetry());
//            telemetry.addData("Number of artifacts", intake.numberOfArtifacts());
//            if (aprilTag != null) {
//                telemetry.addData("AprilTag", aprilTag.getBotpose_MT2());
//            }
//            telemetry.update();
//        }
//    }
//}
