package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.BluePaths.BlueNearGatePaths;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
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
public class BlueNearIntakeGate extends LinearOpMode {
    Drivebase drivebase;
    Shooter shooter;
    Intake intake;
    LimeLight limeLight;
    ServoGate servoGate;
    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    Follower follower;
    private BlueNearGatePaths blueNearGatePaths;
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

        blueNearGatePaths = new BlueNearGatePaths(follower);

        follower.setStartingPose(
                new Pose(
                        20,
                        127,
                        Math.toRadians(143.5)
                )
        );
        setTargets();
        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry,servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Shoot1, 2, telemetry))
                        .add(new TimedShootCommand(shooter, intake, 0.001, telemetry, 1100, servoGate, 0.0, 0.45))
                        .build()
                )
                .add(new TimedShootCommand(shooter, intake, 1.25, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.75, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Intake1, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Shoot2, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 3.25, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Intake2, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Shoot3, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 3.25, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Intake3, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Shoot4, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 3.25, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Intake4, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Shoot5, 2, telemetry))
                        .build()
                )
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.5, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Intake5, 2, telemetry))
                        .build()
                )
                //.add(new BrakeCommand(drivebase, 0.3, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new PedroDriveToCommand(follower, blueNearGatePaths.Shoot6, 2, telemetry))
                        .build()
                )
                //.add(new AimCommand(drivebase, limeLight, telemetry, follower, blueNearPaths.GOAL_X, blueNearPaths.GOAL_Y))
                .add(new TimedShootCommand(shooter, intake, 0.75, telemetry, 1100, servoGate, 0.95, 0.45))
                .add(new PedroDriveToCommand(follower, blueNearGatePaths.Park, 2, telemetry))
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
