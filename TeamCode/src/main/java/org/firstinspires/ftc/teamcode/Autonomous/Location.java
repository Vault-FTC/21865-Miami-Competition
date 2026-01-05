package org.firstinspires.ftc.teamcode.Autonomous;

public class Location {
    public int Forward;
    public int Strafe;
    public int TurnDegrees;

    public Location(int forward, int strafe, int turnDegrees)
    {
        Forward = forward;
        Strafe = strafe;
        TurnDegrees = turnDegrees;
    }
    public Location(int forward, int strafe) {
        this(forward, strafe, 0);
    }
}
