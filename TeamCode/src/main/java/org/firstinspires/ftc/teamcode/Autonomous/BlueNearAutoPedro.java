package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommandWaiting;
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
public class BlueNearAutoPedro extends LinearOpMode {
    Drivebase drive;
    Shooter shooter;
    Intake intake;
    LimeLight LimeLight;
    ServoGate servoGate;
    CommandScheduler scheduler = CommandScheduler.getInstance();
    private BlueNearPaths paths;

    Follower follower;

    double time;

    void setTargets() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivebase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        LimeLight = new LimeLight(hardwareMap,20);
        servoGate = new ServoGate(hardwareMap);
        scheduler.clearRegistry();
        follower = Constants.PedroPathing.createFollower(hardwareMap);

        paths = new BlueNearPaths(follower);

        follower.setStartingPose(
                new Pose(
                        20,
                        123,
                        Math.toRadians(135)
                )
        );

        setTargets();

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new PedroDriveToCommand(follower, paths.BackUpToShootPosition, 3000))
                .add(new TimedShootCommand(shooter, intake, 3.5, telemetry, 1100, servoGate, time))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.PickupFirstRowArtifacts, 3000))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, paths.ShootPosition2, 3000))
                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, time))
                .add(new PedroDriveToCommand(follower, paths.Prepare2ndRowArtifacts, 3000))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, paths.Pickup2ndRowArtifacts, 3000))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, paths.ShootPosition3, 3000))
//                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, time))
//
//                .add(new DriveToCommand(drive, prepareThirdRowArtifacts, telemetry))
//                .add(ParallelCommandGroup.getBuilder()
//                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
//                        .add(new DriveToCommand(drive, collectThirdRowArtifacts, telemetry))
//                        .build()
//                )
//
//                .add(new DriveToCommand(drive, lastLaunchPosition, telemetry))
//                .add(new InstantCommand(() -> PoseStorage.startPose = drive.getPosition()))
//                .add(new TimedShootCommand(shooter, intake, 2.5, telemetry, 1000, servoGate, time))
                .build();


        waitForStart();

        auto.schedule();
        while(opModeIsActive()) {
            time = getRuntime();
            scheduler.run();
            telemetry.addData("Position", drive.getPositionTelemetry());
            drive.update();
            telemetry.update();
        }
    }
}
