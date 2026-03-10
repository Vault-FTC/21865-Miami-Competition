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
import org.firstinspires.ftc.teamcode.Commands.AimCommand;
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
public class BlueFarAuto extends LinearOpMode {
    Drivebase drivebase;
    Shooter shooter;
    Intake intake;
    LimeLight limeLight;
    ServoGate servoGate;
    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    Follower follower;
    private BlueFarPaths blueFarPaths;
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

        blueFarPaths = new BlueFarPaths(follower);

        follower.setStartingPose(
                new Pose(
                        54,
                        6,
                        Math.toRadians(90)
                )
        );
        setTargets();
        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new PedroDriveToCommand(follower, blueFarPaths.Shoot1, 3, telemetry))
                .add(new AimCommand(drivebase, limeLight, telemetry, follower, blueFarPaths.GOAL_X, blueFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1400, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueFarPaths.Intake1, 3, telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, blueFarPaths.Shoot2, 2, telemetry))
                .add(new AimCommand(drivebase, limeLight, telemetry, follower, blueFarPaths.GOAL_X, blueFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 1, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueFarPaths.Intake2, 3,telemetry))
                        .build()
                )

                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueFarPaths.Shoot3, 3,telemetry))
                        .build()
                )
                .add(new AimCommand(drivebase, limeLight, telemetry, follower, blueFarPaths.GOAL_X, blueFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 1, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueFarPaths.Intake2, 3,telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueFarPaths.Shoot3, 3,telemetry))
                        .build()
                )
                .add(new AimCommand(drivebase, limeLight, telemetry, follower, blueFarPaths.GOAL_X, blueFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 1, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueFarPaths.Intake3, 3,telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueFarPaths.Shoot4, 3,telemetry))
                        .build()
                )
                .add(new AimCommand(drivebase, limeLight, telemetry, follower, blueFarPaths.GOAL_X, blueFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 1, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(new PedroDriveToCommand(follower, blueFarPaths.Park, 1, telemetry))
                .build();
        waitForStart();
        auto.schedule();
        while(opModeIsActive()) {
            time = getRuntime();
            commandScheduler.run();
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            PoseStorage.startPose = new Pose2D(DistanceUnit.CM, drivebase.getPosition().getX(DistanceUnit.CM), drivebase.getPosition().getY(DistanceUnit.CM), AngleUnit.DEGREES,drivebase.getPosition().getHeading(AngleUnit.DEGREES) + 90);
            drivebase.update();
            telemetry.update();
        }
    }
}
