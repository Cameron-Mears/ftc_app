package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.ExpansionHubMotor;

public abstract interface Motor
{
    public abstract double getRate();
    public abstract void update();
    public abstract int getPosition();
    public abstract void setTargetPosition(double position);
    public abstract double currentDrawAmperes();
    public abstract CurrentTorqueRelation getCurrentTorqueRelation();
    public abstract void setPower(double power);

    public abstract MotorProperties getProperties();
}
