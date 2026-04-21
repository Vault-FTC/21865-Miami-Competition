package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

@TeleOp
public class TurretPIDTuner extends AbstractOpMode {
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;
    double headingOffset = 0;

    public void setTargets() {
        limelight = new LimeLight(hardwareMap, 20);
    }

    @Override
    public void runOpMode() {
        setTargets();
        startHardware();
        waitForStart();
        turret.resetTimer();

        while (opModeIsActive()) {
//            double error = limelight.getTx();
            double error = turret.getTurretAngle();
            LLResult llResult = limelight.getResult();
            double joystick_y = gamepad1.left_stick_x; // Forward/backward
            double joystick_x = gamepad1.left_stick_y;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation

            if (gamepad1.circleWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if (gamepad1.dpadRightWasPressed()) {
                turret.setkP(turret.getkP() + stepSizes[stepIndex]);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                turret.setkP(turret.getkP() - stepSizes[stepIndex]);
            }
            if (gamepad1.dpadDownWasPressed()) {
                turret.setkD(turret.getkD() + stepSizes[stepIndex]);
            }
            if (gamepad1.dpadUpWasPressed()) {
                turret.setkD(turret.getkD() - stepSizes[stepIndex]);
            }
            //drivebase.drive(joystick_y, joystick_x, joystick_rx, headingOffset);

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
            telemetry.addData("Error", error);
            turret.update(error);

            telemetry.update();
        }
    }
}