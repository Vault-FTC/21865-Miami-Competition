package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
                        20,
                        123,
                        Math.toRadians(135)
                )
        );
        setTargets();
        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new PedroDriveToCommand(follower, redNearPaths.BackUpToShootPosition, 3, telemetry))
                .add(new TimedShootCommand(shooter, intake, 3.5, telemetry, 1100, servoGate, time))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.PickupFirstRowArtifacts, 3, telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, redNearPaths.OpenGate, 2, telemetry))
                .add(new PedroDriveToCommand(follower, redNearPaths.ShootPosition2, 2, telemetry))
                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, time))
                .add(new PedroDriveToCommand(follower, redNearPaths.Prepare2ndRowArtifacts, 3000, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Pickup2ndRowArtifacts, 3000,telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, redNearPaths.ShootPosition3, 3000,telemetry))
                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, time))

                .add(new PedroDriveToCommand(follower, redNearPaths.Prepare3rdRowArtifacts, 3000,telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Pickup3rdRowArtifacts, 3000,telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, redNearPaths.FinalShootPosition, 3000,telemetry))
                .add(new InstantCommand(() -> PoseStorage.startPose = drivebase.getPosition()))
                .add(new TimedShootCommand(shooter, intake, 2.5, telemetry, 1100, servoGate, time))
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
