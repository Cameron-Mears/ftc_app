package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.DriveBase;
import org.firstinspires.ftc.teamcode.control.Motor;

public class Robot
{
    public DriveBase driveBase;
    public Servo servo;

    public Robot(HardwareMap map)
    {
       this.driveBase = new DriveBase(new Motor(map.dcMotor.get("up"), false), new Motor(map.dcMotor.get("down"), false), new Motor(map.dcMotor.get("left"), false), new Motor(map.dcMotor.get("right"), false));
        this.servo = map.servo.get("intake");
    }
}