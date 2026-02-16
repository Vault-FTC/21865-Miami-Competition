package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import java.util.Map;

public class LUT {

    ArrayMap<Double, Double> lookupTable = new ArrayMap<>();
    public LUT()
    {
        add(1, 797.6844);
        add(10, 760.716);
        add(20, 728.532);
        add(30, 705.708);
        add(40, 692.244);
        add(50, 688.14);
        add(60, 693.396);
        add(70, 708.012);
        add(80, 731.988);
        add(90, 765.324);
        add(100, 808.02);
        add(110, 860.076);
        add(120, 921.492);
        add(130, 992.268);
        add(140, 1072.404);
        add(150, 1161.9);
        add(160, 1260.756);
        add(170, 1368.972);
        add(180, 1486.548);
        add(190, 1613.484);
        add(200, 1749.78);
        add(210, 1895.436);
        add(220, 2050.452);
        add(230, 2214.828);
        add(240, 2388.564);
        add(250, 2571.66);
        add(260, 2764.116);
        add(270, 2965.932);
        add(280, 3177.108);
        add(290, 3397.644);
        add(300, 3627.54);
        add(310, 3866.796);
        add(320, 4115.412);
        add(330, 4373.388);
        add(340, 4640.724);
        add(350, 4917.42);
        add(360, 5203.476);
        add(370, 5498.892);
        add(380, 5803.668);
        add(390, 6117.804);
        add(400, 6441.3);
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