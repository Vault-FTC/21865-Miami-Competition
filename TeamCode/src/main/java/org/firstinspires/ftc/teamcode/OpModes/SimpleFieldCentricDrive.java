package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.Autonomous.Location;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;


// ALL SHOOTER SPEEDS ARE IN TICKS/SECOND. DO NOT, I REPEAT DO NOT, USE DEGREES/SECOND
@TeleOp(name = "TeleOp Blue", group = "Teleop")
public class SimpleFieldCentricDrive extends LinearOpMode {

    public LimeLight Limelight;
    Intake intake;
    boolean last_triangle;
    boolean last_up;
    boolean last_down;
    boolean shooting;
    double feedPulseInterval = 0.05; //seconds for feed/pause
    RevBlinkinLedDriver.BlinkinPattern green;
    RevBlinkinLedDriver.BlinkinPattern red;
    Lights light;
    double launchPower = 0;

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
        launchPower = 1200;
        setTargets();
        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        red = RevBlinkinLedDriver.BlinkinPattern.RED;

        waitForStart();
        // poseEstimator.update();
        while (opModeIsActive()) {
            double currentTime = getRuntime();

            double joystick_y = gamepad1.left_stick_y; // Forward/backward
            double joystick_x = gamepad1.left_stick_x;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation

            if (gamepad1.start) {
                drive.resetHeading(0);
            }

            if (!last_triangle && gamepad1.y) {
                shooting = !shooting;
            }

            last_triangle = gamepad1.y;
            if (gamepad1.x) {
                intake.spinIntake(0.675);
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


            if (gamepad1.right_bumper) {
                LLResultTypes.FiducialResult result = Limelight.getResult();
                if (result == null) {

                } else {
                    joystick_rx = joystick_rx - Limelight.getTx() / 2.5;
                    double range = Math.abs(result.getCameraPoseTargetSpace().getPosition().z);
                    gamepad1.rumble(1000);
                    telemetry.addData("fff", range);
                    if (result.getCameraPoseTargetSpace().getPosition().x < 67) {
                        light.setColor(green);
                        if (result.getCameraPoseTargetSpace().getPosition().z >= -2.5) {
                            this.launchPower = 1400;
                            feedPulseInterval = 0.1;
                        }
                        else {
                            this.launchPower = 1800;
                            feedPulseInterval = 0.2;
                        }
                        launcher.setShooterVelocityDynamic(this.launchPower);
                    } else {
                        light.setColor(red);
                    }
                }
            }
            else {
                launcher.setShooterSpeed(0);
            }

            if (gamepad1.back) {
                drive.driveToPosition(new Location(0,0,0), 0 ,telemetry); // Could make this drive to shoot position?
            }
            else {
                drive.drive(joystick_y, joystick_x, joystick_rx);
            }


            last_down = gamepad1.dpad_down;
            last_up = gamepad1.dpad_up;
            telemetry.addData("shootSpeed", launcher.getShooterVelocity());
            telemetry.addData("LaunchPower", this.launchPower);
            telemetry.addData("Position", drive.getPosition());
            if (Limelight.getResult() != null) {
                telemetry.addData("Distance from AprilTag", Limelight.getResult().getCameraPoseTargetSpace().getPosition().z);
            }
            telemetry.update();
            drive.updateValues(telemetry);
        }
    }
}