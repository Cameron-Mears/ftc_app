package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.physicscalculations.PointMass;
import org.firstinspires.ftc.teamcode.robot.Time;


/*
    USE METRIC UNITS OR IT WILL NOT WORK, IF YOU HATE METRIC CHANGE THE STUPID GRAVITY
 */
public class ControlArm
{
    private ControlArm sub;
    private ControlArm parent;
    private double length;
    private Motor motor;
    private double angle;
    private double distanceToCenterOfMass;
    private double mass;
    private double target;
    private PointMass[] significantMasses;

    public ControlArm(PointMass[] significantMasses, Motor motor, double startAngle, double mass, double length)
    {

    }

    public double getRate() //returns the rate of the arm
    {
        return motor.getRate()/motor.getProperties().encoderPlusesPerRevolution/motor.getProperties().gearing * 360;
    }

    /*
        adds on the excepted increment in the the arms angle to the the torque calculation,
        now the motor braking can be set to brake for the future torque of the arm
    */
    public double predictTorque(double xDist, double sum, double angles)
    {
        angles += Math.toRadians(this.angle + (this.getRate() * Time.averageDelta));

        sum += this.mass * 9.8 * ((Math.cos(angles)*this.distanceToCenterOfMass) + xDist);

        xDist += Math.cos(angles) * this.length;
        if (sub == null) return sum;
        return sub.predictTorque(xDist,sum, angles);
    }

    public double getActingTorque(double xDist, double sum, double angles)
    {
        angles += Math.toRadians(this.angle);

        sum += this.mass * 9.8 * ((Math.cos(angles)*this.distanceToCenterOfMass) + xDist);

        xDist += Math.cos(angles) * this.length;
        if (sub == null) return sum;
        return sub.getActingTorque(xDist,sum, angles);
    }

    public void brake()
    {
        target = motor.getPosition();

    }

    public void setTargetRotatation(double theta, boolean keepSubAngles)
    {

    }



}
