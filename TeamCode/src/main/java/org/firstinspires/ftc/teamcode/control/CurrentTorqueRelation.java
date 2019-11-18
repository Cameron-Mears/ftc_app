package org.firstinspires.ftc.teamcode.control;

public abstract interface CurrentTorqueRelation
{
    public abstract void toTorque(double amperage);
    public abstract void toAmperage(double torque);
}
