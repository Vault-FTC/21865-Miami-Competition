package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import java.util.Map;

public class LUT {
    // Assumes hood position is 0.5
    ArrayMap<Double, Double> lookupTableSpeed = new ArrayMap<>();
    ArrayMap<Double, Double> lookupTableHood = new ArrayMap<>();
    public LUT()
    {
        addSpeed(1, 805.6653);
        addSpeed(10, 820.338);
        addSpeed(20, 836.736);
        addSpeed(30, 853.234);
        addSpeed(40, 869.832);
        addSpeed(50, 886.53);
        addSpeed(60, 903.328);
        addSpeed(70, 920.226);
        addSpeed(80, 937.224);
        addSpeed(90, 954.322);
        addSpeed(100, 971.52);
        addSpeed(110, 988.818);
        addSpeed(120, 1006.216);
        addSpeed(130, 1023.714);
        addSpeed(140, 1041.312);
        addSpeed(150, 1059.01);
        addSpeed(160, 1076.808);
        addSpeed(170, 1094.706);
        addSpeed(180, 1112.704);
        addSpeed(190, 1130.802);
        addSpeed(200, 1149);
        addSpeed(210, 1167.298);
        addSpeed(220, 1185.696);
        addSpeed(230, 1204.194);
        addSpeed(240, 1222.792);
        addSpeed(250, 1241.49);
        addSpeed(260, 1260.288);
        addSpeed(270, 1279.186);
        addSpeed(280, 1298.184);
        addSpeed(290, 1317.282);
        addSpeed(300, 1336.48);
        addSpeed(310, 1355.778);
        addSpeed(320, 1358.176);
        addSpeed(330, 1361.674);
        addSpeed(340, 1364.272);
        addSpeed(350, 1383.97);
        addSpeed(360, 1403.000);
        addSpeed(370, 1453.666);
        addSpeed(380, 1473.664);
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