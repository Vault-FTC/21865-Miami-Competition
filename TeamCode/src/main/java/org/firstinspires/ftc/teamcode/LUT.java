package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import java.util.Map;

public class LUT {

    ArrayMap<Double, Double> lookupTable = new ArrayMap<>();
    public LUT()
    {
        add(1, 799.0177);
        add(10, 791.401);
        add(20, 784.572);
        add(30, 779.463);
        add(40, 776.074);
        add(50, 774.405);
        add(60, 774.456);
        add(70, 776.227);
        add(80, 779.718);
        add(90, 784.929);
        add(100, 791.86);
        add(110, 800.511);
        add(120, 810.882);
        add(130, 822.973);
        add(140, 836.784);
        add(150, 852.315);
        add(160, 869.566);
        add(170, 888.537);
        add(180, 909.228);
        add(190, 931.639);
        add(200, 955.77);
        add(210, 981.621);
        add(220, 1009.192);
        add(230, 1038.483);
        add(240, 1069.494);
        add(250, 1102.225);
        add(260, 1136.676);
        add(270, 1172.847);
        add(280, 1210.738);
        add(290, 1250.349);
        add(300, 1291.68);
        add(310, 1334.731);
        add(320, 1379.502);
        add(330, 1425.993);
        add(340, 1474.204);
        add(350, 1524.135);
        add(360, 1575.786);
        add(370, 1629.157);
        add(380, 1684.248);
        add(390, 1741.059);
        add(400, 1799.59);
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