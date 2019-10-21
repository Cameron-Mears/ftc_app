package org.firstinspires.ftc.teamcode.control;


/*
 Class for arms all torque calculations are recursive, so for multi joint arms only one function
 call is needed for the base arm, the program will calculate all the torques on the arm
 including any sub arms
*/

import org.firstinspires.ftc.teamcode.physicscalculations.*;

public class Arm
{
    private static final double FORCE_GRAVITY = 9.80;

    public final Arm subArm; //if there is another arm joined
    private PID pid;
    private Motor motor;
    public final double distanceToCenterOfMass;
    public final double mass; //total mass
    public final double massOfBeam; //mass of the beam
    public PointMass[] attachments;
    public final double length;

    public Arm(Motor motor, Arm subArm, double mass, double massOfBeam, double length, double distanceToCenterOfMass, double K_i, double K_d, double K_p)
    {
        pid = new PID(motor.encoder, 0, 0,0 );
        this.motor = motor;
        this.subArm = subArm;
        this.massOfBeam = massOfBeam;
        this.length = length;
        this.mass = mass;
        this.distanceToCenterOfMass = distanceToCenterOfMass;

    }
    //x and y are relative to base of arm
    public void initPointMasses(PointMass ...masses)
    {
        this.attachments = masses;
    }

    public double calculate_torque_compensation()
    {
        double sum = this.sum_torque(0 , 0, 0, 0, 0);
        PointMass point = this.getCenterOfMass(0, 0, 0, 0, 0);
        double compensationPower = (sum * this.motor.free_rpm * Motor.NEWTON_METER_POWER_CONSERVION)/this.motor.max_power;
        return compensationPower;
    };

    //make sure when calling this method sum is zero
    public double sum_torque(double sum, double angles, int depth, double distancesX, double distancesY)
    {
        depth ++;
        double armAngle = this.computeAngle();
        angles += armAngle;

        double calculationAngle = 0;
        if (depth % 2 != 0) calculationAngle = 180 - calculationAngle; //odd call 180 - angle

        calculationAngle = Math.toRadians(calculationAngle);
        sum += Math.sin((Math.PI)/2 - calculationAngle) * (this.distanceToCenterOfMass + Math.hypot(distancesX, distancesY)) * FORCE_GRAVITY; //calculate moment on arm
        distancesX += Math.cos(calculationAngle) * this.length;
        distancesY += Math.sin(calculationAngle) * this.length;
        if (this.subArm != null) sum += this.subArm.sum_torque(sum, angles, depth, distancesX, distancesY); //sum the torque acting on subjoints
        return sum;
    }
    /*
       returns an array of size 2 , contaning 3 numbers, first the x point, then y point
       of the center of mass of the form the given arm, and third the total mass

    */

    public PointMass getCenterOfMass(double xTotalLength, double yTotalLength, double xVecsMass, double yVecsMass, double masses)
    {
        double angle = Math.toRadians(this.computeAngle());

        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        xVecsMass += ((cos * this.distanceToCenterOfMass) + xTotalLength) * this.mass;
        yVecsMass += ((sin * this.distanceToCenterOfMass) + yTotalLength) * this.mass;


        xTotalLength += cos * this.length;
        yTotalLength += sin * this.length;
        masses += this.mass;
        if (this.subArm == null) return new PointMass(xVecsMass/xTotalLength, yVecsMass/yTotalLength, masses);
        else return subArm.getCenterOfMass(xTotalLength, yTotalLength, xVecsMass, yVecsMass, masses);
    };

    public double computeAngle()
    {
        this.motor.encoder.update();
        double encoderPosition = this.motor.encoder.getPosition();
        double angleDegrees = ((encoderPosition % this.motor.encoderPlusesPerRevolution)/this.motor.encoderPlusesPerRevolution) * 360;
        return angleDegrees;
    }

    public void setTargetAngle(double angleDegrees)
    {
        this.pid.setTargetPos((int)(angleDegrees/360 * this.motor.encoderPlusesPerRevolution));
    }

    public void update()
    {
        this.motor.motor.setPower(this.calculate_torque_compensation() + this.pid.calculate());
    }



}