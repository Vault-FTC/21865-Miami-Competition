package org.firstinspires.ftc.teamcode.Autonomous.BlueAutons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.BluePaths.BlueNearGatePaths;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.BluePaths.BlueNearPathsNew;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.BrakeCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PedroDriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OpModes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;


    @Autonomous(name ="New Blue Auto Near Gate")
    public class blueAutoNewGate extends AbstractOpMode {
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

            BlueNearPathsNew blueNearGatePaths = new BlueNearPathsNew(follower);

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
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot1, 2, telemetry))
                                    .add(new TimedShootCommand(shooter, intake, 0.5, telemetry, 1100, servoGate, 0.0, 0.45))
                                    .add(new IntakeCommand(intake, 1.0, telemetry,servoGate))
                                    .build())
                    .add(new TimedShootCommand(shooter, intake, 1.5, telemetry, 1100, servoGate, 0.95, 0.45))


                                    .add(ParallelCommandGroup.getBuilder()
                    .add(new PedroDriveToCommand(follower, blueNearGatePaths.SetForFirstSpike, 2, telemetry))
                            .add(new IntakeCommand(intake, 1.0, telemetry,servoGate))
                    .build())

                    .add(ParallelCommandGroup.getBuilder()
                    .add(new PedroDriveToCommand(follower, blueNearGatePaths.CollectFirstSpike, 2, telemetry))
                            .add(new IntakeCommand(intake, 1.0, telemetry,servoGate))
                    .build())

                    .add(ParallelCommandGroup.getBuilder()
                    .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot2, 2, telemetry))
                    .build())

                    .add(ParallelCommandGroup.getBuilder()
                    .add(new PedroDriveToCommand(follower, blueNearGatePaths.SetSpike2, 2, telemetry))
                    .build())

                    .add(ParallelCommandGroup.getBuilder()
                    .add(new PedroDriveToCommand(follower, blueNearGatePaths.CollectSpike2, 2, telemetry))
                    .build())

                    .add(ParallelCommandGroup.getBuilder()
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot3, 2, telemetry))
                            .build())

                    .add(ParallelCommandGroup.getBuilder()
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.GateCollec1, 2, telemetry))
                            .build())

                    .add(ParallelCommandGroup.getBuilder()
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot1, 2, telemetry))
                            .build())

                    .add(ParallelCommandGroup.getBuilder()
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot1, 2, telemetry))
                            .build())

                    .add(ParallelCommandGroup.getBuilder()
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot1, 2, telemetry))
                            .build())

                    .add(ParallelCommandGroup.getBuilder()
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot1, 2, telemetry))
                            .build())

                    .add(ParallelCommandGroup.getBuilder()
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot1, 2, telemetry))
                            .build())

                    .add(ParallelCommandGroup.getBuilder()
                            .add(new PedroDriveToCommand(follower, blueNearGatePaths.MoveToShoot1, 2, telemetry))
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
            }
        }
    }

