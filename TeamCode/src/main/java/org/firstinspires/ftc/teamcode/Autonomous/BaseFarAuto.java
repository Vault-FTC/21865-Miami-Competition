package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@Autonomous(name = "Blue Far", group = "Blue Team")
public class BaseFarAuto extends BaseNearAuto {
    Location doNotHitWall = new Location(30, 0, 25);
    Location farShootPosition = new Location(30, 0, 40);
    Location firstPickupPosition = new Location(80, 0, 95);
    Location firstPickupPosition2 = new Location(70,-130, 95);
    Location parkPosition = new Location(88, -10, 95);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    double time;
    void setTargets() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivebase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        scheduler.clearRegistry();
        LimeLight = new LimeLight(hardwareMap,24);
        setTargets();

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new DriveToCommand(drive, doNotHitWall, telemetry))
                .add(new DriveToCommand(drive, farShootPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive,LimeLight, telemetry))
                .add(new TimedShootCommand(shooter, intake, 4, telemetry, 1500, servoGate, time))
                .add(new DriveToCommand(drive, firstPickupPosition, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 3, telemetry, servoGate))
                        .add(new DriveToCommand(drive, firstPickupPosition2, telemetry))
                        .build()
                )
                .add(new DriveToCommand(drive, farShootPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive,LimeLight, telemetry))
                .add(new TimedShootCommand(shooter, intake, 4, telemetry, 1500, servoGate, time))
                .add(new DriveToCommand(drive, parkPosition, telemetry))
//                .add(new InstantCommand(() -> PoseStorage.startPose = drive.getPosition()))
                .build();

        waitForStart();

        auto.schedule();
        while(opModeIsActive()) {
            time = getRuntime();
            scheduler.run();
            telemetry.update();
        }
    }
}
