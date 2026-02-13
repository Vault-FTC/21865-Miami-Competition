package org.firstinspires.ftc.teamcode.Autonomous;

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
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@Autonomous (name = "Blue Near", group = "Blue Team")
public class BaseNearAuto extends LinearOpMode {
    Drivebase drive;
    Shooter shooter;
    Intake intake;
    LimeLight LimeLight;
    ServoGate servoGate;
    Pose2D startPosition = new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES, 135);
//    Location launchPosition = new Location(36, -203, 42);
//    Location collectFirstRowArtifacts = new Location(-70, -60, 43);
//    Location hitGate = new Location(-60, -105, -48);
//    Location hitGate2 = new Location(-70, -110, -48);
//    Location prepareSecondRowArtifacts = new Location(-145,-80, 43);
//    Location collectSecondRowArtifacts = new Location(-88, -135, 43);
//    Location prepareCollectThirdRowArtifacts = new Location(-184,-124, 43);
//    Location collectionThirdRowArtifacts = new Location(-124,-175, 43);
//    Location lastLaunchPosition = new Location(-110, 20, 0);
//    Location leaveZonePosition = new Location(-80, -70, 43);

    Location launchPosition = AutonomousPositions.launchPosition;


    Location launchPosition1 = new Location(-40, -40, -135);
    Location collectFirstRowArtifacts = AutonomousPositions.collectFirstRowArtifacts;
    Location prepareSecondRowArtifacts = AutonomousPositions.prepareSecondRowArtifacts;
    Location collectSecondRowArtifacts = AutonomousPositions.collectSecondRowArtifacts;
    Location collectThirdRowArtifacts = AutonomousPositions.collectionThirdRowArtifacts;
    Location prepareThirdRowArtifacts = AutonomousPositions.prepareCollectThirdRowArtifacts;
    Location hitGate = AutonomousPositions.hitGate;
    Location hitGate2 = AutonomousPositions.hitGate2;
    Location leaveZonePosition = AutonomousPositions.leaveZonePosition;
    Location lastLaunchPosition = AutonomousPositions.lastLaunchPosition;



    CommandScheduler scheduler = CommandScheduler.getInstance();
    Command auto;
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
        drive.setCurrentPose(AutonomousPositions.BLUE_START_POSITION);

        setTargets();

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new DriveToCommand(drive, launchPosition, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new TimedShootCommand(shooter, intake, 3.5, telemetry, 1100, servoGate, time))
                        .add(new DriveToCommandWaiting(drive, launchPosition, telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new DriveToCommand(drive, collectFirstRowArtifacts, telemetry))
                        .build()
                )
                .add(new DriveToCommand(drive, launchPosition, telemetry))
                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, time))
                .add(new DriveToCommand(drive, prepareSecondRowArtifacts, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new DriveToCommand(drive, collectSecondRowArtifacts, telemetry))
                        .build()
                )
                .add(new DriveToCommand(drive, prepareSecondRowArtifacts, telemetry))
                .add(new DriveToCommand(drive, launchPosition, telemetry))
                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, time))

                .add(new DriveToCommand(drive, prepareThirdRowArtifacts, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new DriveToCommand(drive, collectThirdRowArtifacts, telemetry))
                        .build()
                )

                .add(new DriveToCommand(drive, lastLaunchPosition, telemetry))
                .add(new InstantCommand(() -> PoseStorage.startPose = drive.getPosition()))
                .add(new TimedShootCommand(shooter, intake, 2.5, telemetry, 1000, servoGate, time))
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
