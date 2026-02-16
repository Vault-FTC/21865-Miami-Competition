package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx rb = hardwareMap.get(DcMotorEx.class, "rb");
        DcMotorEx lb = hardwareMap.get(DcMotorEx.class, "lb");

        waitForStart();

        while(opModeIsActive())
        {
            doMotor(gamepad1.a, lf, "LF");
            doMotor(gamepad1.b, lb, "LB");
            doMotor(gamepad1.x, rf, "RF");
            doMotor(gamepad1.y, rb, "RB");
            telemetry.update();
        }
    }

    public void doMotor(boolean button, DcMotorEx motor, String name)
    {
        if(button)
        {
            motor.setPower(0.4);
            telemetry.addData(name , " : active");
        }
        else
        {
            motor.setPower(0);
            telemetry.addData(name , " : off");
        }
    }
}
