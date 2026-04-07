package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.RedPaths.RedFarPaths;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.AimCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PedroDriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OpModes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

@Autonomous
public class RedFarAuto extends AbstractOpMode {
    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    Follower follower;
    private RedFarPaths redFarPaths;
    double time;

    void setTargets() {
    }
    @Override
    public void runOpMode() throws InterruptedException {
        startHardware();
        limelight = new LimeLight(hardwareMap,20);
        commandScheduler.clearRegistry();
        follower = Constants.PedroPathing.createFollower(hardwareMap);

        redFarPaths = new RedFarPaths(follower);

        follower.setStartingPose(
                new Pose(
                        90,
                        6,
                        Math.toRadians(90)
                )
        );
        setTargets();
        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new PedroDriveToCommand(follower, redFarPaths.Shoot1, 3, telemetry))
                .add(new AimCommand(drivebase, limelight, telemetry, follower, redFarPaths.GOAL_X, redFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 4, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redFarPaths.Intake1, 3, telemetry))
                        .build()
                )
                .add(new PedroDriveToCommand(follower, redFarPaths.Shoot2, 2, telemetry))
                .add(new AimCommand(drivebase, limelight, telemetry, follower, redFarPaths.GOAL_X, redFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redFarPaths.Intake2, 3,telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redFarPaths.Shoot3, 3,telemetry))
                        .build()
                )
                .add(new AimCommand(drivebase, limelight, telemetry, follower, redFarPaths.GOAL_X, redFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 2.5, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redFarPaths.Intake2, 3,telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redFarPaths.Shoot3, 3,telemetry))
                        .build()
                )
                .add(new AimCommand(drivebase, limelight, telemetry, follower, redFarPaths.GOAL_X, redFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 2.5, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redFarPaths.Intake3, 3,telemetry))
                        .build()
                )
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redFarPaths.Shoot4, 3,telemetry))
                        .build()
                )
                .add(new AimCommand(drivebase, limelight, telemetry, follower, redFarPaths.GOAL_X, redFarPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 2.5, telemetry, 1450, servoGate, 0.7, 0.7))
                .add(new PedroDriveToCommand(follower, redFarPaths.Park, 3, telemetry))
                .build();
        waitForStart();
        auto.schedule();
        while(opModeIsActive()) {
            time = getRuntime();
            commandScheduler.run();
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            PoseStorage.startPose = new Pose2D(DistanceUnit.CM, drivebase.getPosition().getX(DistanceUnit.CM), drivebase.getPosition().getY(DistanceUnit.CM), AngleUnit.DEGREES,drivebase.getPosition().getHeading(AngleUnit.DEGREES) + 90);
            drivebase.update();
            telemetry.update();
        }
    }
}
