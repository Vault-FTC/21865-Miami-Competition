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
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.FieldConstants;
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
    Pose2D startPosition;
    Location launchPosition = new Location(-30.5, -30.5, 0);
    Location collectFirstRowArtifacts = new Location(-30.5, -152.4, 43);
    Location hitGate = new Location(0, -152.4, -48);
    Location hitGate2 = new Location(0, -152.4, -48);
    Location prepareSecondRowArtifacts = new Location(30.5,-91.4, 43);
    Location collectSecondRowArtifacts = new Location(30.5, -152.4, 43);
    Location prepareCollectThirdRowArtifacts = new Location(91.4,-91.4, 43);
    Location collectionThirdRowArtifacts = new Location(91.4,-152.4, 43);
    Location lastLaunchPosition = new Location(-110, 20, 0);
    Location leaveZonePosition = new Location(-80, -70, 43);


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
//        drive.setCurrentPose(startPosition);

        setTargets();

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new DriveToCommand(drive, launchPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive, LimeLight, telemetry))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1350, servoGate, time))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new DriveToCommand(drive, collectFirstRowArtifacts, telemetry))
                        .build()
                )
                .add(new DriveToCommand(drive, hitGate, telemetry))
                .add(new DriveToCommand(drive, launchPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive,LimeLight,telemetry))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1350, servoGate, time))
                .add(new DriveToCommand(drive, prepareSecondRowArtifacts, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new DriveToCommand(drive, collectSecondRowArtifacts, telemetry))
                        .build()
                )
                .add(new DriveToCommand(drive, prepareSecondRowArtifacts, telemetry))
                .add(new DriveToCommand(drive, hitGate2, telemetry))
                .add(new DriveToCommand(drive, launchPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive,LimeLight,telemetry))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1350, servoGate, time))

//                .add(new DriveToCommand(drive, prepareCollectThirdRowArtifacts, telemetry))
//                .add(ParallelCommandGroup.getBuilder()
//                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
//                        .add(new DriveToCommand(drive, collectionThirdRowArtifacts, telemetry))
//                        .build()
//                )
//
//                .add(new DriveToCommand(drive, lastLaunchPosition, telemetry))
//                .add(new TimedShootCommand(shooter, intake, 2.5, telemetry, 1350, servoGate, time))
                .add(new DriveToCommand(drive, leaveZonePosition, telemetry))
//                .add(new InstantCommand(() -> PoseStorage.startPose = drive.getPosition()))
                .build();


        waitForStart();

        auto.schedule();
        while(opModeIsActive()) {
            time = getRuntime();
            scheduler.run();
            telemetry.addData("Position", drive.getPositionTelemetry());
            telemetry.update();
        }
    }
}
