package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.physicscalculations.PointMass;
import org.firstinspires.ftc.teamcode.robot.Time;

public class ControlArm
{
    private ControlArm sub;
    private ControlArm parent;
    private int length;
    private ArmMotor motor;
    private double angle;
    private double distanceToCenterOfMass;
    private double mass;
    private double target;
    private PointMass[] significantMasses;

    public ControlArm()
    {

    }

    public double getRate()
    {
        return this.motor.getRate()/this.motor.getProperties().encoderPlusesPerRevolution/this.motor.getProperties().gearing * 360;
    }

    /*
        adds on the excepted increment in the the arms angle to the the torque calculation,
        now the motor braking can be set to brake for the future torque of the arm
    */
    public double predictTorque(double xDist, double yDist, double sum, double angles)
    {
        angles += Math.toRadians(this.angle + (this.getRate() * Time.averageDelta));

        sum += Math.cos(angles) * this.mass * ( Math.hypot(xDist, yDist) + this.distanceToCenterOfMass) * 9.8;

        xDist += Math.cos(this.angle) * this.length;
        yDist += Math.sin(this.angle) * this.length;

        return sub.predictTorque(xDist, yDist, sum, angles);
    }

    public double getActingTorque(double xDist, double sum, double angles)
    {
        angles += Math.toRadians(this.angle);

        sum += this.mass * 9.8 * ((Math.cos(angle)*this.distanceToCenterOfMass) + xDist);

        xDist += Math.cos(angles) * this.length;
        if (sub == null) return sum;
        return sub.getActingTorque(xDist,sum, angles);
    }

    public void brake()
    {
        target = motor.getPosition();

    }

    public void setTaregtRotatation(double theta, boolean keepSubAngles)
    {

    }



}
