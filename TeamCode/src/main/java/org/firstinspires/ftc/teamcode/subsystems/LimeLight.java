package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.VisionFixQuality;

public class LimeLight {
    public static final double VISION_WEIGHT = 0.3; // 30% vision, 70% odometry
    Limelight3A limelight;
    VisionFixQuality visionFixQuality;
    Drivebase drivebase;
    int apriltag;

    public LimeLight(HardwareMap hardwareMap, int apriltag, Drivebase drivebase)
    {
        this.apriltag = apriltag;
        this.drivebase = drivebase;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.visionFixQuality = new VisionFixQuality();
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public LLResult getLatestValidResult() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }
        return result;
    }

    public void update() {
        GoBildaPinpointDriver odo = drivebase.getOdo();
        odo.update();
        limelight.updateRobotOrientation(odo.getHeading(AngleUnit.DEGREES));
        LLResult result = getLatestValidResult();

        if (result == null) return;

        if (!visionFixQuality.isGoodFix(result, odo.getPosition())) return;

        if (visionFixQuality.isGoodFix(result, odo.getPosition())) {
            Pose3D visionPose = result.getBotpose_MT2();
            double newX = odo.getPosX(DistanceUnit.CM) * (1-VISION_WEIGHT) + visionPose.getPosition().x * VISION_WEIGHT;
            double newY = odo.getPosY(DistanceUnit.CM) * (1-VISION_WEIGHT) + visionPose.getPosition().y * VISION_WEIGHT;
            double newH = odo.getHeading(AngleUnit.RADIANS) * (1-VISION_WEIGHT) + visionPose.getOrientation().getYaw(AngleUnit.RADIANS);
            drivebase.setCurrentPose(new Pose2D(DistanceUnit.CM, newX, newY, AngleUnit.RADIANS, newH));
        }
    }

    public Pose3D getBotPose() {
        LLResult r = getLatestValidResult();
        if (r == null) return null;

        return r.getBotpose_MT2();
    }

    public double getTx()
    {
        LLResult r = getLatestValidResult();
        if (r == null) return 0.0;

        double tx = r.getTx();
        return tx * 0.3;
    }
    public Pose3D getFieldPose() {
        LLResult result = getLatestValidResult();
        if (result != null) {
            return result.getBotpose_MT2();
        }
        else{
            return null;
        }
    }
}
