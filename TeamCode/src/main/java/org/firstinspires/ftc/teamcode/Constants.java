package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Constants {


    public static final double CLOSE_SHOOT_SPEED = 1100;
    public static final Pose2D BLUE_LAUNCH = new Pose2D(DistanceUnit.CM, -110, 10, AngleUnit.RADIANS, 0);
    public static final Pose2D RED_LAUNCH = new Pose2D(DistanceUnit.CM, -110, 10, AngleUnit.RADIANS, 0);
    public static final Pose2D BLUE_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -152.4, -152.4, AngleUnit.RADIANS, 0);
    public static final Pose2D RED_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -152.4, 152.4, AngleUnit.RADIANS, 0);



    public static final class Drive {
        public static final double trackEndpointHeadingMaxDistance = 12.0;
        public static final double calculateTargetHeadingMinDistance = 15.0;
        public static final double defaultFollowRadius = 15;
    }

    public static class PedroPathing {
        public static FollowerConstants followerConstants = new FollowerConstants()
                .mass(11.5);

        public static MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(1)
                .rightFrontMotorName("rf")
                .rightRearMotorName("lb")
                .leftRearMotorName("rb")
                .leftFrontMotorName("lf")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .xVelocity(85.53);

        public static PinpointConstants localizerConstants = new PinpointConstants()
                .forwardPodY(-1.5175)
                .strafePodX(20.571207)
                .distanceUnit(DistanceUnit.CM)
                .hardwareMapName("odo")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(followerConstants, hardwareMap)
                    .pathConstraints(pathConstraints)
                    .mecanumDrivetrain(driveConstants)
                    .pinpointLocalizer(localizerConstants)
                    .build();
        }
    }
}
