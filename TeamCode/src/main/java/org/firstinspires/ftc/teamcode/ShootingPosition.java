package org.firstinspires.ftc.teamcode;

public enum ShootingPosition {
THREE_EIGHTS(1000),
HALF(1100),
FIVE_EIGHTS(1200),
THREE_QUARTERS(1500),
EIGHTY(1000),
NINETY(5400),
NINETY_FIVE(1400),
FULL(6000);

public final double speed;

ShootingPosition(double val)
{
    speed = val;
}
}

