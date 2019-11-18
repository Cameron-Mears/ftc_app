package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.ExpansionHubMotor;
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
    public void setTargetPosition(double position)
    {

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
    private ExpansionHubMotor motor;
    private long lastCallTime;
    private MotorProperties properties;
    public ArmMotor()
    {

    }

    @Override
    public double currentDrawAmperes()
    {
        return 0;
    }

    @Override
    public CurrentTorqueRelation getCurrentTorqueRelation()
    {
        return null;
    }
}
