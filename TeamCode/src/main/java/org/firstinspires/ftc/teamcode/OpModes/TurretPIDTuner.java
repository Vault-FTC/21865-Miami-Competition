package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

@TeleOp
public class TurretPIDTuner extends AbstractBlueOpMode {
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};
    int stepIndex = 2;
    double headingOffset = 0;

    @Override
    public void runOpMode() {
        startHardware();
        setTargets();
        drivebase.setCurrentPose(PoseStorage.startPose);
        waitForStart();
        turret.resetTimer();

        while (opModeIsActive()) {
//            double error = limelight.getTx();
            double error = turret.getTurretAngle();
            LLResult aprilTag = drivebase.update(limelight);
            LLResult llResult = limelight.getResult();
            double joystick_y = gamepad1.left_stick_x; // Forward/backward
            double joystick_x = gamepad1.left_stick_y;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation

            if (aprilTag != null) {
                drivebase.setCurrentPose(aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).x,
                        aprilTag.getBotpose_MT2().getPosition().toUnit(DistanceUnit.CM).y);
                telemetry.addData("BotPose", aprilTag.getBotpose_MT2().getPosition());
            } else if (gamepad1.circleWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            } else if (gamepad1.dpadRightWasPressed()) {
                turret.setkP(turret.getkP() + stepSizes[stepIndex]);
            } else if (gamepad1.dpadLeftWasPressed()) {
                turret.setkP(turret.getkP() - stepSizes[stepIndex]);
            } else if (gamepad1.dpadDownWasPressed()) {
                turret.setkD(turret.getkD() + stepSizes[stepIndex]);
            } else if (gamepad1.dpadUpWasPressed()) {
                turret.setkD(turret.getkD() - stepSizes[stepIndex]);
            } else {
                drivebase.drive(joystick_y, joystick_x, joystick_rx);
            }

//            if (llResult == null)
//            {
//                // no tag
//            telemetry.addLine("No Tag Detected. Stopping Turret Motor");
//            }
//            else
//            {
//                for(LLResultTypes.FiducialResult res: llResult.getFiducialResults())
//                {
//                    // found a tag
////                    res.getFiducialId();
//                    telemetry.addData("Tag ID", res.getFiducialId());
//
//                }
//            }
            telemetry.addLine("-----------------------------------------------------------");
            telemetry.addData("Tuning P", "%.5f (D-Pad L/R", turret.getkP());
            telemetry.addData("Tuning D", "%.5f (D-Pad U/D", turret.getkD());
            telemetry.addData("Step Size", "%.5f (B Button", stepSizes[stepIndex]);
            turret.update(error);
            drivebase.update();
            telemetry.addData("OdoPosition" , drivebase.getPositionTelemetry());
            telemetry.update();
        }
    }
}