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

    public LLResult getResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result;
//                for(LLResultTypes.FiducialResult res : result.getFiducialResults()) {
//                    if(res.getFiducialId() == apriltag)
//                    {
//                        return res;
//                    }
            }
        }
        return null;
    }

    public LLResult getEitherResult()
    {
        LLResult res = limelight.getLatestResult();
        apriltag = 20;
        res = getResult();
        if (res == null) {
            apriltag = 24;
            res = getResult();
        }
        return res;
    }
    public double getTx()
    {
        LLResult res = getResult();
        if(res != null) {
            if(Math.abs(res.getTx()) > 0.25)
            {
                return res.getTx() * 0.05;
            }
        }
        return 0;
    }
    public Pose3D getFieldPose() {
        LLResult result = getResult();
        if (result != null) {
            return result.getBotpose_MT2();
        }
        else{
            return null;
        }
    }
}
