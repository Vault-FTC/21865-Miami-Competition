package org.firstinspires.ftc.teamcode.OldAutonomousClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.InstantCommand;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@Autonomous(name = "Red Park", group = "Blue Team")
public class RedFarPark extends LinearOpMode {
    Location parkPosition = new Location(133, 60, -90);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    double time;
    Drivebase drive;
    Shooter shooter;
    Intake intake;
    ServoGate servoGate;
    void setTargets() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivebase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        servoGate = new ServoGate(hardwareMap);
        scheduler.clearRegistry();
        drive.setCurrentPose(AutonomousPositions.BLUE_FAR_START_POSITION);
        setTargets();

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()

                .add(new DriveToCommand(drive, parkPosition, telemetry))
                .add(new InstantCommand(() -> PoseStorage.startPose = drive.getPosition()))
                .build();

        waitForStart();

        auto.schedule();
        while(opModeIsActive()) {
            scheduler.run();
            drive.update();
            telemetry.update();
        }
    }
}
