package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class ServoGate extends Subsystem {
    private Servo servoGate;
   //
    public ServoGate(HardwareMap hardwareMap) {
        servoGate = hardwareMap.get(Servo.class, "servoGate");
    }
    public void closeGate()    {
        servoGate.setPosition(0.9);
    }
    public void openGate()  {
        servoGate.setPosition(0.5);
    }
}