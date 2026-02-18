package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import java.util.Map;

public class LUT {
    // Assumes hood position is 0.5
    ArrayMap<Double, Double> lookupTable = new ArrayMap<>();
    public LUT()
    {
        add(1, 800.9567);
        add(10, 817.388);
        add(20, 835.816);
        add(30, 854.424);
        add(40, 873.212);
        add(50, 892.18);
        add(60, 911.328);
        add(70, 930.656);
        add(80, 950.164);
        add(90, 969.852);
        add(100, 989.72);
        add(110, 1009.768);
        add(120, 1029.996);
        add(130, 1050.404);
        add(140, 1070.992);
        add(150, 1091.76);
        add(160, 1112.708);
        add(170, 1133.836);
        add(180, 1155.144);
        add(190, 1176.632);
        add(200, 1198.3);
        add(210, 1220.148);
        add(220, 1242.176);
        add(230, 1264.384);
        add(240, 1286.772);
        add(250, 1309.34);
        add(260, 1332.088);
        add(270, 1355.016);
        add(280, 1378.124);
        add(290, 1401.412);
        add(300, 1424.88);
        add(310, 1448.528);
        add(320, 1472.356);
        add(330, 1496.364);
        add(340, 1520.552);
        add(350, 1544.92);
        add(360, 1569.468);
        add(370, 1594.196);
        add(380, 1619.104);
    }
    
    public double getSpeed(double distance)
    {
        double lowerLimit = 0;
        double higherLimit = 0;
        for(Map.Entry<Double, Double> value: lookupTable.entrySet())
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
        return lookupTable.valueAt(lookupTable.size() - 1);
    }
    
    public void add(double distance, double speed)
    {
        lookupTable.put(distance, speed);
    }
}