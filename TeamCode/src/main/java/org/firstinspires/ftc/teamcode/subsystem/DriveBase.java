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

        this.rightPID = new PID(right.encoder, 0, 0, 0);
        this.leftPID = new PID(left.encoder, 0, 0, 0);
        this.downPID = new PID(down.encoder, 0, 0, 0);
        this.upPID = new PID(up.encoder, 0, 0, 0);
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

        this.left.motor.setPower(-leftPower);
        this.right.motor.setPower(rightPower);
        this.up.motor.setPower(-upPower);
        this.down.motor.setPower(downPower);
    }

    public void autoDrive()
    {
        this.up.motor.setPower(this.upPID.calculate());
        this.down.motor.setPower(this.downPID.calculate());
        this.left.motor.setPower(this.leftPID.calculate());
        this.right.motor.setPower(this.rightPID.calculate());
    }
    public void translateVertical(double meters)
    {
        this.upPID.setTargetPos((int)((meters/WHEEL_CIRCUMFRENCE) * this.up.encoderPlusesPerRevolution));
        this.downPID.setTargetPos((int)((meters/WHEEL_CIRCUMFRENCE) * this.down.encoderPlusesPerRevolution));
    }

    public void translateHorizental(double meters)
    {
        this.leftPID.setTargetPos((int)((meters/WHEEL_CIRCUMFRENCE) * this.left.encoderPlusesPerRevolution));
        this.rightPID.setTargetPos((int)((meters/WHEEL_CIRCUMFRENCE) * this.right.encoderPlusesPerRevolution));
    }

    public void rotate(double degrees)
    {
        double meters = TURNING_RADIUS/degrees;
        this.leftPID.setTargetPos((int)((meters/WHEEL_CIRCUMFRENCE) * this.left.encoderPlusesPerRevolution));
        this.rightPID.setTargetPos((int)((-meters/WHEEL_CIRCUMFRENCE) * this.right.encoderPlusesPerRevolution));
        this.upPID.setTargetPos((int)((-meters/WHEEL_CIRCUMFRENCE) * this.up.encoderPlusesPerRevolution));
        this.downPID.setTargetPos((int)((meters/WHEEL_CIRCUMFRENCE) * this.down.encoderPlusesPerRevolution));
    }


}