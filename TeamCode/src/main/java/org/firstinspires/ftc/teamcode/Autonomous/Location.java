package org.firstinspires.ftc.teamcode.Autonomous;

public class Location {
    public double Forward;
    public double Strafe;
    public int TurnDegrees;

    public Location(double forward, double strafe, int turnDegrees)
    {
        Forward = forward;
        Strafe = strafe;
        TurnDegrees = turnDegrees;
    }
    public Location(int forward, int strafe) {
        this(forward, strafe, 0);
    }
}
