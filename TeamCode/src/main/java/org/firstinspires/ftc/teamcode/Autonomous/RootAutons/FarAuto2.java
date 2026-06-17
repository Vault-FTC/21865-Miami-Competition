package org.firstinspires.ftc.teamcode.Autonomous.RootAutons;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPaths2;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.BrakeCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommandDynamicAim;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PedroDriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OpModes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

public abstract class FarAuto2 extends AbstractOpMode {

    protected abstract FarPaths2 createPaths(Follower follower);

    @Override
    public void runOpMode() throws InterruptedException {
        startHardware();
        CommandScheduler commandScheduler = CommandScheduler.getInstance();
        commandScheduler.clearRegistry();

        Follower follower = Constants.PedroPathing.createFollower(hardwareMap);
        FarPaths2 paths = createPaths(follower);
        follower.setStartingPose(paths.getStartingPose());
        drivebase.setCurrentPose(
                paths.getStartingPose().getX() * 2.54,
                paths.getStartingPose().getY() * 2.54,
                paths.getStartingPose().getHeading()
        );

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                // Shoot 1
                .add(new DriveToCommandDynamicAim(follower, paths::buildShoot1, 3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new TimedShootCommand(shooter, intake, 3.75, telemetry, 1450, servoGate, 0.9, 0.7))
                        .add(new BrakeCommand(drivebase, 1, telemetry))
                        .build()
                )
                // Spike1 — intake while driving to spike piece
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Spike1, 2, telemetry))
                        .build()
                )
                // Shoot 2
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot2, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1450, servoGate, 0.9, 0.7))
                        .add(new BrakeCommand(drivebase, 1, telemetry))
                        .build()
                )
                // Intake 2
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake2, 2, telemetry))
                        .build()
                )
                // Shoot 3
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot3, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1450, servoGate, 0.9, 0.7))
                        .add(new BrakeCommand(drivebase, 1, telemetry))
                        .build()
                )
                // Intake 3
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake3, 3, telemetry))
                        .build()
                )
                // Shoot 4
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot4, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1450, servoGate, 0.9, 0.7))
                        .add(new BrakeCommand(drivebase, 1, telemetry))
                        .build()
                )
                // Intake 4
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake4, 3, telemetry))
                        .build()
                )
                // Shoot 5
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot5, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1450, servoGate, 0.9, 0.7))
                        .add(new BrakeCommand(drivebase, 1, telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, paths.Park, 3, telemetry))
                .build();

        waitForStart();
        auto.schedule();
        while (opModeIsActive()) {
            commandScheduler.run();
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            PoseStorage.startPose = follower.getPose();
            intake.update();
            drivebase.update();
            telemetry.update();
        }
    }
}
