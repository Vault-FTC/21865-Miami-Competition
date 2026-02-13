package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import java.util.Map;

public class LUT {

    ArrayMap<Double, Double> lookupTable = new ArrayMap<>();
    public LUT()
    {
        add(80, 1017.352);
        add(90, 1019.701);
        add(100, 1024.29);
        add(110, 1031.119);
        add(120, 1040.188);
        add(130, 1051.497);
        add(140, 1065.046);
        add(150, 1080.835);
        add(160, 1098.864);
        add(170, 1119.133);
        add(180, 1141.642);
        add(190, 1166.391);
        add(200, 1193.38);
        add(210, 1222.609);
        add(220, 1254.078);
        add(230, 1287.787);
        add(240, 1323.736);
        add(250, 1361.925);
        add(260, 1402.354);
        add(270, 1445.023);
        add(280, 1489.932);
        add(290, 1537.081);
        add(300, 1586.47);
        add(310, 1638.099);
        add(320, 1691.968);
        add(330, 1748.077);
        add(340, 1806.426);
        add(350, 1867.015);
        add(360, 1929.844);
        add(370, 1994.913);
        add(380, 2062.222);
        add(390, 2131.771);
        add(400, 2203.56);
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