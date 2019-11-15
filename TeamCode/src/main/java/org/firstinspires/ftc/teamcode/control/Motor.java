package org.firstinspires.ftc.teamcode.control;

public abstract interface Motor
{
    public abstract double getRate();
    public abstract void update();
    public abstract int getPosition();
    public abstract void setPower(double power);

    public abstract MotorProperties getProperties();
}
