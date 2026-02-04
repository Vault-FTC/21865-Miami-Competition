package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;

@TeleOp
public class PIDTuner extends OpMode {
    Intake intake;
    ServoGate servoGate;
    public DcMotorEx flywheelMotor;
    double highVelocity = 1800;
    double lowVelocity = 1000;
    double off = 0;
    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

//    double F = 10;
//    double P = 20;
//    double velocity = 900;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        intake = new Intake(hardwareMap);
        servoGate = new ServoGate(hardwareMap);
//        if (gamepad1.triangleWasPressed()) {
//            velocity += stepSizes[stepIndex];
//        }
//        if (gamepad1.squareWasPressed()) {
//            velocity -= stepSizes[stepIndex];
//        }
        if (gamepad1.triangleWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            }
            else {
                if (curTargetVelocity == lowVelocity) {
                    curTargetVelocity = off;
                }
                else {
                    curTargetVelocity = highVelocity;
                }
            }
        }

        if (gamepad1.circleWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.square) {
            servoGate.openGate();
            intake.spinIntake(0.95);
        }
        else {
            servoGate.closeGate();
            intake.spinIntake(0);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

//        flywheelMotor.setVelocity(velocity);
        flywheelMotor.setVelocity(curTargetVelocity);
        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

//        telemetry.addData("Target Velocity", velocity);
//        telemetry.addData("Current Velocity", "%.2f", curVelocity);
//        telemetry.addData("Error", "%.2f", error);
//        telemetry.addLine("-----------------------");
//        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
//        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
//        telemetry.addData("Step Size", "%.4f (B-Button)", stepSizes[stepIndex]);

telemetry.addData("Target Velocity", curTargetVelocity);
telemetry.addData("Current Velocity", "%.2f", curVelocity);
telemetry.addData("Error", "%.2f", error);
telemetry.addLine("-----------------------");
telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
telemetry.addData("Step Size", "%.4f (B-Button)", stepSizes[stepIndex]);
    }
}
