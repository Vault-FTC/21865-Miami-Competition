package org.firstinspires.ftc.teamcode.Autonomous.RedAutons;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.RedPaths.RedNearPaths;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PedroDriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OpModes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

@Autonomous
public class RedNearTunnel extends AbstractOpMode {
    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    Follower follower;
    private RedNearPaths redNearPaths;
    double time;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = new LimeLight(hardwareMap,20);
        commandScheduler.clearRegistry();
        follower = Constants.PedroPathing.createFollower(hardwareMap);

        redNearPaths = new RedNearPaths(follower);

        follower.setStartingPose(
                new Pose(
                        124.000,
                        127.000,
                        Math.toRadians(36.5)
                )
        );
        setTargets();
        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry,servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Shoot1, 2, telemetry))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 2.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.75, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Intake1, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Gate1, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Shoot2, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Intake2, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.Shoot3, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 3.25, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.GateIntake3, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.GateShoot4, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 3.25, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.GateIntake3, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, redNearPaths.GateShoot4, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(new PedroDriveToCommand(follower, redNearPaths.Park, 2, telemetry))
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .build();
        waitForStart();
        auto.schedule();
        while(opModeIsActive()) {
            time = getRuntime();
            commandScheduler.run();
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            PoseStorage.startPose = new Pose2D(DistanceUnit.CM, drivebase.getPosition().getX(DistanceUnit.CM), drivebase.getPosition().getY(DistanceUnit.CM), AngleUnit.DEGREES,(drivebase.getPosition().getHeading(AngleUnit.DEGREES)) + 90);
            drivebase.update();
            telemetry.update();
        }
    }
}