package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.BluePaths.TwentySevenPathsBlue;
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

@Autonomous(name ="27 BLUE NEAR AUTO")

public class TwentySevenNearBlueAuto extends AbstractOpMode {
    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    Follower follower;
    double time;


    @Override
    public void runOpMode() throws InterruptedException {
        startHardware();
        limelight = new LimeLight(hardwareMap, 20);
        commandScheduler.clearRegistry();
        Constants.PedroPathing.driveConstants.setXVelocity(30);
        Constants.PedroPathing.driveConstants.setYVelocity(30);
        follower = Constants.PedroPathing.createFollower(hardwareMap);

        TwentySevenPathsBlue twentySevenNearBlueAuto = new TwentySevenPathsBlue(follower);

        follower.setStartingPose(
                new Pose(
                        20,
                        127,
                        Math.toRadians(143.5)
                )
        );
        setTargets();

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(
                        ParallelCommandGroup.getBuilder()
                                .add(new PedroDriveToCommand(follower, TwentySevenPathsBlue.MoveToShoot1, 2, telemetry))
                                .add(new TimedShootCommand(shooter, intake, 2, telemetry, 1100, servoGate, 1, 0.45))
                                .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                                .build())
                .add(new TimedShootCommand(shooter, intake, 1.5, telemetry, 1100, servoGate, 0.95, 0.45))

                .add(ParallelCommandGroup.getBuilder()
                        .add(new PedroDriveToCommand(follower, TwentySevenPathsBlue.SetFirstSpike, 2, telemetry))
                        .add(new TimedShootCommand(shooter, intake, 0.5, telemetry, 1100, servoGate, 0.5, 0.45))
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .build())
                .add(new TimedShootCommand(shooter, intake, 1.5, telemetry, 1100, servoGate, 0.95, 0.45))


                .add(ParallelCommandGroup.getBuilder()
                        .add(new PedroDriveToCommand(follower, TwentySevenPathsBlue.CollectFirstSpike, 2, telemetry))
                        .add(new TimedShootCommand(shooter, intake, 0.5, telemetry, 1100, servoGate, 0.5, 0.45))
                        .add(new IntakeCommand(intake, 1.0, telemetry, servoGate))
                        .add(new TimedShootCommand(shooter, intake, 1.5, telemetry, 1100, servoGate, 0.95, 0.45))
                        .build())
                .build();

        waitForStart();
        auto.schedule();
        while (opModeIsActive()) {
            time = getRuntime();
            commandScheduler.run();
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            PoseStorage.startPose = new Pose2D(DistanceUnit.CM, drivebase.getPosition().getX(DistanceUnit.CM), drivebase.getPosition().getY(DistanceUnit.CM), AngleUnit.DEGREES, (drivebase.getPosition().getHeading(AngleUnit.DEGREES)) + 90);
            drivebase.update();
            telemetry.update();
            intake.update();
            shooter.update();
        }
    }
}

