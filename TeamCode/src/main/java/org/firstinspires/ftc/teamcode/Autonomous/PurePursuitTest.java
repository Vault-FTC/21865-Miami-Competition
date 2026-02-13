package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PurePursuit.Path;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitCore;
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
        PurePursuitCore pathFollower = new PurePursuitCore(() -> drive.getOdo().getPosition(), drive, telemetry);
        drive.resetHeading(0);

        double followRadius = 10;

        Path path = Path.getBuilder()
                .addWaypoint(new Waypoint(0,0, followRadius))
                .addWaypoint(new Waypoint(20, 20, followRadius))
                .build();

        pathFollower.setFollowPath(path);


        waitForStart();

        while (opModeIsActive() && !pathFollower.finishedFollowing()) {
            drive.getOdo().update();
            pathFollower.followPath();
            telemetry.addData("Robot Position", drive.getPositionTelemetry());
            telemetry.update();
        }

        drive.drive(0,0,0);

    }
}
