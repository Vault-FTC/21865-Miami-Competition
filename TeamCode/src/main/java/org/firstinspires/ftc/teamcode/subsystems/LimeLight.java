package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public LLResultTypes.FiducialResult getEitherResult()
    {
        LLResultTypes.FiducialResult res;
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
        LLResultTypes.FiducialResult res = getResult();
        if(res != null) {
            if(Math.abs(res.getTargetXDegrees()) > 0.25)
            {
                return res.getTargetXDegrees() * 0.05;
            }
        }
        return 0;
    }
}
