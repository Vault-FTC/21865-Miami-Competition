package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PurePursuit.Path;
import org.firstinspires.ftc.teamcode.PurePursuit.Waypoint;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@TeleOp
public class PurePursuitTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivebase drive = new Drivebase(hardwareMap);

        Waypoint[] pathPoints = new Waypoint[] {
                new Waypoint(0, 0, 0),
                new Waypoint(50, 0, 0),
                new Waypoint(100, 50,0)
        };

        Path path = new Path(pathPoints);

        drive.setFollowPath(path);

        waitForStart();

        while (opModeIsActive() && !drive.finishedFollowing()) {
            drive.followPath();
            telemetry.addData("Robot Position", drive.getPosition());
        }
        drive.drive(0,0,0);
        telemetry.update();
    }
}
