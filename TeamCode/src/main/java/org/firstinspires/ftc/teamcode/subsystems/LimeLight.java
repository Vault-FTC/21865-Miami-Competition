package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimeLight {
    Limelight3A limelight;
    int apriltag;

    public LimeLight(HardwareMap hardwareMap, int apriltag)
    {
        this.apriltag = apriltag;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public LLResultTypes.FiducialResult getResult()
    {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                for(LLResultTypes.FiducialResult res : result.getFiducialResults()) {
                    if(res.getFiducialId() == apriltag)
                    {
                        return res;
                    }
                }
            }
        }
        return null;
    }

    public void updateHeading(double degrees)
    {
        limelight.updateRobotOrientation(degrees);
    }

    public LLResult getEitherResult()
    {
        LLResult res = limelight.getLatestResult();
        if(res.isValid()) {
            return res;
        }
        else
            return null;
    }
    public double getTx()
    {
        LLResultTypes.FiducialResult res = getResult();
        if(res != null) {
            if(Math.abs(res.getTargetXDegrees()) > 0.25)
            {
                return res.getTargetXDegrees() * 0.05;
            }
        }
        return 0;
    }
    public Pose3D getFieldPose() {
        LLResultTypes.FiducialResult result = getResult();
        if (result != null) {
            return result.getRobotPoseFieldSpace();
        }
        else{
            return null;
        }
    }
}
