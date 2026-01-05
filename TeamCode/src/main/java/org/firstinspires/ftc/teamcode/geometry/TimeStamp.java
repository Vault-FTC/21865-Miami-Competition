package org.firstinspires.ftc.teamcode.geometry;

import java.security.CodeSigner;
import java.util.Timer;

public class TimeStamp {
    private double lastTime;
    private double Timer = 0;

    public TimeStamp(){
        reset();
    }

    /**
     * @return time since last reset.
     * Does not reset automatically
     */
    public double getDt(){
        return getCurrentTime() - lastTime;
    }

    public double getLastTime(){
        return lastTime;
    }

    public double getCurrentTime(){
        return Timer;//.();
    }

    public void reset(){
        lastTime = getCurrentTime();
    }
}
