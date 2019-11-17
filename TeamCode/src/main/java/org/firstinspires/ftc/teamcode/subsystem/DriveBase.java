package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.control.Motor;
import org.firstinspires.ftc.teamcode.control.PID;

public class DriveBase
{
    private static final double WHEEL_CIRCUMFRENCE = 0.12;
    private static final double TURNING_RADIUS = 0.1;
    private Motor left;
    private Motor right;
    private Motor up;
    private Motor down;

    private PID leftPID;
    private PID rightPID;
    private PID downPID;
    private PID upPID;

    public DriveBase(Motor up, Motor down, Motor left, Motor right)
    {
        this.up = up;
        this.down = down;
        this.left = left;
        this.right = right;
    }

    public void teleOpdrive(Gamepad gamepad)
    {
        double leftPower, rightPower, upPower, downPower;
        leftPower = rightPower = upPower = downPower = 0;

        leftPower += gamepad.left_stick_x;
        rightPower += gamepad.left_stick_x;
        upPower += gamepad.left_stick_y;
        downPower += gamepad.left_stick_y;

        leftPower -= gamepad.right_stick_x;
        rightPower += gamepad.right_stick_x;
        upPower += gamepad.right_stick_x;
        downPower -= gamepad.right_stick_x;

    }

    public void autoDrive()
    {

    }



}