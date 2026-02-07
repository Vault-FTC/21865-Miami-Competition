package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PurePursuit.Path;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuiter;
import org.firstinspires.ftc.teamcode.PurePursuit.Rotation2d;
import org.firstinspires.ftc.teamcode.PurePursuit.Vector2d;
import org.firstinspires.ftc.teamcode.PurePursuit.Waypoint;
import org.firstinspires.ftc.teamcode.PurePursuit.WaypointGenerator;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@Autonomous
public class PurePursuitTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivebase drive = new Drivebase(hardwareMap);
        PurePursuiter pursuiter = new PurePursuiter(drive::getPosition, drive, telemetry, drive.getOdo());
        drive.resetHeading(0);

        double followRadius = 150;

        Path path = Path.getBuilder()
                .addWaypoint(new Waypoint(0,0, followRadius, new Rotation2d(0), new Rotation2d( 0)))
                .addWaypoint(new Waypoint(0, 10, followRadius, new Rotation2d(0), new Rotation2d(0)))
                .build();

        pursuiter.setFollowPath(path);


        waitForStart();

        while (opModeIsActive()) {
            pursuiter.followPath();
            telemetry.addData("Robot Position", drive.getPositionTelemetry());
            telemetry.update();
        }

    }
}
