//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.subsystems.CaseModes;
//import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;
//
//@TeleOp
//public class TestOpMode extends LinearOpMode {
//
//    TestSubsystem test;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        test = new TestSubsystem(hardwareMap);
//
//        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
//        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
//        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
//        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            if (gamepad1.a) {
//                test.setMode(CaseModes.SHOOT);
//            }
//            else if (gamepad1.b){
//                test.setMode(CaseModes.REVERSE);
//            }
//            else
//            {
//                test.setMode(CaseModes.OFF);
//            }
//
//            doMotor(gamepad1.a, frontLeftMotor, "frontLeftMotor");
//            doMotor(gamepad1.x, frontRightMotor, "frontRightMotor");
//            doMotor(gamepad1.b, backLeftMotor, "backLeftMotor");
//            doMotor(gamepad1.y, backRightMotor, "backRightMotor");
//
//
//            test.update();
//            telemetry.update();
//        }
//    }
//
//    public void doMotor(boolean button, DcMotorEx motor, String name)
//    {
//        if(button)
//        {
//            motor.setPower(0.4);
//            telemetry.addData(name , " : active");
//        }
//        else
//        {
//            motor.setPower(0);
//            telemetry.addData(name , " : off");
//        }
//    }
//}
