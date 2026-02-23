package org.firstinspires.ftc.teamcode.OldAutonomousClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@Autonomous(name = "Blue Far", group = "Blue Team")
public class BaseFarAuto extends BaseNearAuto {
    Location doNotHitWall = new Location(152.4, 15.25, -180);
    Location farShootPosition = new Location(152, 15.25, -150); // 163
    Location firstPickupPosition = new Location(90, 50, -90);
    Location firstPickupPosition2 = new Location(145, 90, -90);
    Location parkPosition = new Location(133, 60, -90);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    double time;
    void setTargets() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivebase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        servoGate = new ServoGate(hardwareMap);
        scheduler.clearRegistry();
//        LimeLight = new LimeLight(hardwareMap,24);
        drive.setCurrentPose(AutonomousPositions.BLUE_FAR_START_POSITION);
        setTargets();

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
//                .add(new DriveToCommand(drive, doNotHitWall, telemetry))
                .add(new DriveToCommand(drive, farShootPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive,LimeLight, telemetry))
                .add(new TimedShootCommand(shooter, intake, 8, telemetry, 1750, servoGate, time))
//                .add(new DriveToCommand(drive, firstPickupPosition, telemetry))
//                .add(ParallelCommandGroup.getBuilder()
//                        .add(new IntakeCommand(intake, 3, telemetry, servoGate))
//                        .add(new DriveToCommand(drive, firstPickupPosition2, telemetry))
//                        .build()
//                )
//                .add(new DriveToCommand(drive, farShootPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive,LimeLight, telemetry))
//                .add(new TimedShootCommand(shooter, intake, 8, telemetry, 1750, servoGate, time))
                .add(new DriveToCommand(drive, parkPosition, telemetry))
                .add(new InstantCommand(() -> PoseStorage.startPose = drive.getPosition()))
                .build();

        waitForStart();

        auto.schedule();
        while(opModeIsActive()) {
            time = getRuntime();

            if (time > 15) {
                scheduler.run();
            }
            drive.update();
            telemetry.update();
        }
    }
}
