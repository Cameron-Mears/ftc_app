package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.physicscalculations.PointMass;

public class ArmMotor implements Motor
{
    @Override
    public double getRate()
    {
        return 0;
    }

    @Override
    public void update()
    {
        long now = System.currentTimeMillis();
        int newPos = this.getPosition();
    }

    @Override
    public int getPosition()
    {
        return motor.getCurrentPosition();
    }

    @Override
    public void setPower(double power)
    {
        motor.setPower(power);
    }

    @Override
    public MotorProperties getProperties()
    {
        return null;
    }

    private double rangeOfMotion;
    private double stallTorque;
    private double outputPower;
    private double rate;
    private double lastPos;
    private DcMotor motor;
    private long lastCallTime;
    private MotorProperties properties;
    public ArmMotor()
    {

    }



}
