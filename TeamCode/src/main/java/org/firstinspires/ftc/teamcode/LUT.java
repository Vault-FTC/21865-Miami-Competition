package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import java.util.Map;

public class LUT {
    // Assumes hood position is 0.5
    ArrayMap<Double, Double> lookupTableSpeed = new ArrayMap<>();
    ArrayMap<Double, Double> lookupTableHood = new ArrayMap<>();
    public LUT()
    {
        addSpeed(1, 800.9567);
        addSpeed(10, 817.388);
        addSpeed(20, 835.816);
        addSpeed(30, 854.424);
        addSpeed(40, 873.212);
        addSpeed(50, 892.18);
        addSpeed(60, 911.328);
        addSpeed(70, 930.656);
        addSpeed(80, 950.164);
        addSpeed(90, 969.852);
        addSpeed(100, 989.72);
        addSpeed(110, 1009.768);
        addSpeed(120, 1029.996);
        addSpeed(130, 1050.404);
        addSpeed(140, 1070.992);
        addSpeed(150, 1091.76);
        addSpeed(160, 1112.708);
        addSpeed(170, 1133.836);
        addSpeed(180, 1155.144);
        addSpeed(190, 1176.632);
        addSpeed(200, 1198.3);
        addSpeed(210, 1220.148);
        addSpeed(220, 1242.176);
        addSpeed(230, 1264.384);
        addSpeed(240, 1286.772);
        addSpeed(250, 1309.34);
        addSpeed(260, 1332.088);
        addSpeed(270, 1355.016);
        addSpeed(280, 1378.124);
        addSpeed(290, 1401.412);
        addSpeed(300, 1424.88);
        addSpeed(310, 1448.528);
//        addSpeed(320, 1472.356);
//        addSpeed(330, 1496.364);
//        addSpeed(340, 1520.552);
//        addSpeed(350, 1544.92);
//        addSpeed(360, 1569.468);
//        addSpeed(370, 1594.196);
//        addSpeed(380, 1619.104);
        addSpeed(320, 1450);
        addSpeed(330, 1450);
        addSpeed(340, 1450);
        addSpeed(350, 1450);
        addSpeed(360, 1450);
        addSpeed(370, 1450);
        addSpeed(380, 1450);
    }

    public void LUTHood () {

    }
    public double getSpeed(double distance)
    {
        double lowerLimit = 0;
        double higherLimit = 0;
        for(Map.Entry<Double, Double> value: lookupTableSpeed.entrySet())
        {
            if(distance - 10 < value.getKey() && lowerLimit == 0)
            {
                lowerLimit = value.getValue();
            }
            if(distance < value.getKey())
            {
                double remainder = distance % 10;
                remainder = remainder * 0.1;
                double lowerLimVal = lowerLimit * (1 - remainder);
                higherLimit = value.getValue();
                double higherLimVal = higherLimit * remainder;
                return lowerLimVal + higherLimVal;
            }
        }
        return lookupTableSpeed.valueAt(lookupTableSpeed.size() - 1);
    }
    
    public void addSpeed(double distance, double speed)
    {
        lookupTableSpeed.put(distance, speed);
    }
    public void addAngle(double distance, double angle) {
        lookupTableHood.put(distance, angle);
    }
}