package org.firstinspires.ftc.teamcode.physicscalculations;

public class PointMass
{
    private double x;
    private double y;
    private double mass;

    public PointMass(double x, double y, double mass)
    {
        this.x = x;
        this.y = y;
        this.mass = mass;
    }

    public double getMass()
    {
        return mass;
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }
}
