package org.firstinspires.ftc.teamcode.Autonomous.RootAutons;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommandDynamicAim;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PedroDriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OpModes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

public abstract class FarAuto extends AbstractOpMode {

    protected abstract Alliance getAlliance();

    @Override
    public void runOpMode() throws InterruptedException {
        startHardware();
        CommandScheduler commandScheduler = CommandScheduler.getInstance();
        commandScheduler.clearRegistry();

        Follower follower = Constants.PedroPathing.createFollower(hardwareMap);
        FarPaths paths = new FarPaths(follower, getAlliance());
        follower.setStartingPose(paths.getStartingPose());

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new DriveToCommandDynamicAim(follower, paths::buildShoot1, 3, telemetry))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1400, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake1, 3, telemetry))
                        .build()
                )
                .add(new DriveToCommandDynamicAim(follower, paths::buildShoot2, 2, telemetry))
                .add(new TimedShootCommand(shooter, intake, 1, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake2, 3, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot3, 3, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 1, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake2, 3, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot3, 3, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 1, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake3, 3, telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, paths.Park, 3, telemetry))
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
            intake.update();
            drivebase.update();
            telemetry.update();
        }
    }
}
