package org.firstinspires.ftc.teamcode.control;


/*
 Class for arms all torque calculations are recursive, so for multi joint arms only one function
 call is needed for the base arm, the program will calculate all the torques on the arm
 including any sub arms
*/

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private double rate;
    private double gearing;
    public final double length;
    private double relAngle;
    public Arm(Motor motor, Arm subArm, double mass, double massOfBeam, double length, double distanceToCenterOfMass, double K_i, double K_d, double K_p)
    {

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



    //make sure when calling this method sum is zero
    public double computeTorque(double sum, double angles, int depth, double distancesX, double distancesY)
    {
        depth ++;
        double armAngle = this.computeAngle();

        double calculationAngle = 0;
        if (depth % 2 != 0) calculationAngle = 180 - calculationAngle; //odd call 180 - angle
        if (depth != 0)this.relAngle = calculationAngle; //save rel angle from when calling from this instance
        double sin, cos;
        calculationAngle = Math.toRadians(this.relAngle + armAngle);
        sin = Math.sin(calculationAngle);
        cos = Math.cos(calculationAngle);
        distancesX += cos * this.distanceToCenterOfMass;
        distancesY += sin * this.distanceToCenterOfMass;
        sum += Math.sin((Math.PI)/2 - calculationAngle) * (Math.hypot(distancesX, distancesY)) * FORCE_GRAVITY; //calculate moment on arm
        distancesX -= cos * this.distanceToCenterOfMass;
        distancesY -= sin * this.distanceToCenterOfMass;
        distancesX += cos* this.length;
        distancesY += sin * this.length;
        if (this.subArm != null)
        {
            sum += this.subArm.computeTorque(sum, angles, depth, distancesX, distancesY); //sum the torque acting on subjoints
        }
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
        return 0;
    }









}