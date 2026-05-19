package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearPaths;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PedroDriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OpModes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

public abstract class NearAuto extends AbstractOpMode {

    protected abstract Alliance getAlliance();

    @Override
    public void runOpMode() throws InterruptedException {
        startHardware();
        CommandScheduler commandScheduler = CommandScheduler.getInstance();
        commandScheduler.clearRegistry();

        Follower follower = Constants.PedroPathing.createFollower(hardwareMap);
        NearPaths paths = new NearPaths(follower, getAlliance());
        follower.setStartingPose(paths.getStartingPose());

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Shoot1, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 2.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.75, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake1, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Gate1, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Shoot2, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake2, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Shoot3, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Intake3, 2, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Shoot4, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(new PedroDriveToCommand(follower, paths.Park, 2, telemetry))
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
