package org.firstinspires.ftc.teamcode.Autonomous.RootAutons;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.NearIntakeGatePaths;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommandDynamicAim;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OpModes.AbstractOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

public abstract class TestIntakePowerDraw extends AbstractOpMode {

    protected abstract Alliance getAlliance();

    @Override
    public void runOpMode() throws InterruptedException {
        startHardware();
        CommandScheduler commandScheduler = CommandScheduler.getInstance();
        commandScheduler.clearRegistry();

        Follower follower = Constants.PedroPathing.createFollower(hardwareMap);
        NearIntakeGatePaths paths = new NearIntakeGatePaths(follower, getAlliance());
        follower.setStartingPose(paths.getStartingPose());

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 25.0, telemetry, servoGate))
                        .add(new DriveToCommandDynamicAim(follower, paths::buildShoot1, 2, telemetry))
                        .build()
                )
                .build();
        waitForStart();
        auto.schedule();
        while (opModeIsActive()) {
            commandScheduler.run();
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.addData("IntakePowerDraw:", intake.currentDraw());
            PoseStorage.startPose = follower.getPose();
            drivebase.update();
            telemetry.update();
            intake.update();
        }
    }
}
