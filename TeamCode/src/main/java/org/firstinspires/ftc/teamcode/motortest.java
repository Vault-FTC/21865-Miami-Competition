package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
@Disabled
public class motortest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {



        DcMotorEx frMotor = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx flMotor = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx brmotor = hardwareMap.get(DcMotorEx.class, "rb");
        DcMotorEx blmotor = hardwareMap.get(DcMotorEx.class, "lb");

        waitForStart();

        while (opModeIsActive())
        {
       if (gamepad1.a)
       {frMotor.setPower(0.4);}
       else if (gamepad1.b)
        {flMotor.setPower(0.4);}
      else if (gamepad1.x)
        {brmotor.setPower(0.4);}
       else if (gamepad1.y)
        {blmotor.setPower(0.4);}
        else {
           frMotor.setPower(0);
           flMotor.setPower(0);
           brmotor.setPower(0);
           blmotor.setPower(0);
       }
       }
    }
}
