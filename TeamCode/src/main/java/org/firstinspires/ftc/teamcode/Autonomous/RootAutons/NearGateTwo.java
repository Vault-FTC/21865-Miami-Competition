package org.firstinspires.ftc.teamcode.Autonomous.RootAutons;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearGateTwoPaths;
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

public abstract class NearGateTwo extends AbstractOpMode {

    protected abstract Alliance getAlliance();

    @Override
    public void runOpMode() throws InterruptedException {
        startHardware();
        CommandScheduler commandScheduler = CommandScheduler.getInstance();
        commandScheduler.clearRegistry();

        Follower follower = Constants.PedroPathing.createFollower(hardwareMap);
        NearGateTwoPaths paths = new NearGateTwoPaths(follower, getAlliance());
        follower.setStartingPose(paths.getStartingPose());

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot1, 2, telemetry))
                        .add(new TimedShootCommand(shooter, intake, 0.001, telemetry, 1100, servoGate, 0.0, 0.45))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 1.1, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.SpikeIntake1, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 0.75, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot2, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 0.6, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.GateIntake1, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new BrakeCommand(drivebase, 1.0, telemetry))
                        .add(new IntakeCommand(intake, 2.0, telemetry, servoGate))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 0.75, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot3, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 0.6, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.GateIntake2, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new BrakeCommand(drivebase, 1.0, telemetry))
                        .add(new IntakeCommand(intake, 2.0, telemetry, servoGate))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 0.75, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot4, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 0.6, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.SpikeIntake2, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new BrakeCommand(drivebase, 1.0, telemetry))
                        .add(new IntakeCommand(intake, 2.0, telemetry, servoGate))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 0.75, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot5, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 0.6, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.SpikeIntake3, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot6, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 0.7, telemetry, 1100, servoGate, 0.95, 0.45))
                .build();

        waitForStart();
        auto.schedule();
        while (opModeIsActive()) {
            commandScheduler.run();
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            PoseStorage.startPose = new Pose2D(
                    DistanceUnit.CM,
                    drivebase.getPosition().getX(DistanceUnit.CM),
                    drivebase.getPosition().getY(DistanceUnit.CM),
                    AngleUnit.DEGREES,
                    drivebase.getPosition().getHeading(AngleUnit.DEGREES) + 90
            );
            drivebase.update();
            telemetry.update();
            intake.update();
        }
    }
}
