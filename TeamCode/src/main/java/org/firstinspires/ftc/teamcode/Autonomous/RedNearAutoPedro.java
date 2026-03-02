package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PedroDriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class RedNearAutoPedro extends LinearOpMode {
    Drivebase drivebase;
    Shooter shooter;
    Intake intake;
    LimeLight limeLight;
    ServoGate servoGate;
    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    Follower follower;
    private RedNearPaths redNearPaths;
    double time;

    void setTargets() {
    }
    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new Drivebase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        limeLight = new LimeLight(hardwareMap,20);
        servoGate = new ServoGate(hardwareMap);
        commandScheduler.clearRegistry();
        follower = Constants.PedroPathing.createFollower(hardwareMap);

        redNearPaths = new RedNearPaths(follower);

        follower.setStartingPose(
                new Pose(
                        125.26829268292684,
                        125.07317073170731,
                        Math.toRadians(45)
                )
        );
        setTargets();
        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new PedroDriveToCommand(follower, redNearPaths.Shoot1, 3, telemetry))
                .add(new TimedShootCommand(shooter, intake, 3.5, telemetry, 1100, servoGate, 0.95))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Intake1, 3, telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, redNearPaths.Shoot2, 2, telemetry))
                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, 0.95))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Intake2, 3000,telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, redNearPaths.Shoot3, 3000,telemetry))
                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, 0.95))

                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Intake4, 3000,telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, redNearPaths.Shoot4, 3000,telemetry))
                .add(new InstantCommand(() -> PoseStorage.startPose = new Pose2D(DistanceUnit.CM, drivebase.getPosition().getX(DistanceUnit.CM), drivebase.getPosition().getY(DistanceUnit.CM), AngleUnit.DEGREES,(drivebase.getPosition().getHeading(AngleUnit.DEGREES)) + 90)))
                .add(new TimedShootCommand(shooter, intake, 2.5, telemetry, 1100, servoGate, 0.95))
                .build();
        waitForStart();
        auto.schedule();
        while(opModeIsActive()) {
            time = getRuntime();
            commandScheduler.run();
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            drivebase.update();
            telemetry.update();
        }
    }
}
