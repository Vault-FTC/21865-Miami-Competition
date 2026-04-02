package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.LUT;

public class Shooter extends Subsystem {
    private static final double CLOSE_DIST_CM = 140;
    private static final double MID_DIST_CM = 270;
    private static final double CLOSE_HOOD = 0.15;
    private static final double MID_HOOD = 0.5; //0.2
    private static final double FAR_HOOD = 0.7; //0.5
    private final DcMotorEx shooter;
    private ServoGate servoGate;
    private Drivebase driveBase;
    private final Servo hood;
    private Drivebase drive;
    private Intake intake;

    private final LUT lut = new LUT();
    double distance, speed;
    CaseModes currentMode = CaseModes.OFF;
    double kP = 0.02;
    double kD = 0.0015;

    Pose2D goal = Constants.BLUE_CENTER_GOAL;

    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(250, 0, 0, 15);

    public Shooter(HardwareMap hardwareMap, Drivebase driveBase, ServoGate servoGate) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        hood = hardwareMap.get(Servo.class, "hood");
        this.servoGate = servoGate;
        this.driveBase = driveBase;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void update() {
        double angleError = Drivebase.angleToGoal(drive.getPosition(), goal);
        double joystick_rx = -gamepad1.right_stick_x; // Rotation
        double velocityDeg = drive.getOdo().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        switch(currentMode){
            case OFF:
                shooter.setVelocity(0);
                servoGate.closeGate();
                break;
            case SHOOT:
                intake.setState(CaseModes.ON);
                double errorDeg = (angleError+0.05) * (180 / Math.PI);
                joystick_rx = joystick_rx + errorDeg * kP - velocityDeg * kD;
                servoGate.openGate();
                if (Math.abs((angleError+0.05) * ((180/Math.PI))) < 1 && getShooterVelocity() >= distanceToSpeed(distance)) {
                    intake.setState(CaseModes.SIXTY_PERCENT_SPEED);
                    gamepad1.rumble(1000);
                }
                break;
            case SHOOT_GATE_CLOSED:
                servoGate.closeGate();
            case REVERSE:
                servoGate.openGate();
                intake.setState(CaseModes.REVERSE);
                shooter.setPower(-0.3);
                break;
        }
    }

    public void setState(CaseModes s) {
        currentMode = s;
    }
    public double distanceToSpeed(double distanceCm) {
        speed = lut.getSpeed(distanceCm);
        distance = distanceCm;
        return speed;
    }
    public double distanceToHoodPosition(double distanceCm)
    {
        if (distanceCm <= CLOSE_DIST_CM) {
            return CLOSE_HOOD;
        } else if (distanceCm <= MID_DIST_CM) {
            return MID_HOOD;
        } else {
            return FAR_HOOD;
        }
    }
    public void setShooterSpeedNear(double speed){
        pidfCoefficients = new PIDFCoefficients(250, 0, 0, 15);
        shooter.setVelocity(speed);
    }
    public void setShooterSpeedFar(double speed) {
        pidfCoefficients = new PIDFCoefficients(350, 0, 0, 15.2); // p:440 f:14  for new shooter if we need to change it
        shooter.setVelocity(speed);
    }
    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }
    public String telemetryUpdate() {
        return "Servo Position: " + hood.getPosition()
                + " \n Shooter Speed: " + getShooterVelocity() + " \n " + "Target Speed/Vel: " + distance + ":" + speed;
    }
    public double getShooterVelocity()
    {
        return shooter.getVelocity();
    }

    double degreesToPosition(double degrees) {
        return 0.15 + (degrees - 15) / (75 - 15) * (0.70 - 0.15);
    }

}
