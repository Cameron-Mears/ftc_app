package org.firstinspires.ftc.teamcode.control;


/*
 Class for arms all torque calculations are recursive, so for multi joint arms only one function
 call is needed for the base arm, the program will calculate all the torques on the arm
 including any sub arms
*/

public class Arm
{
    private static final double FORCE_GRAVITY = 9.8;

    private Arm subArm; //if there is another arm joined
    private PID pid;
    private Motor motor;
    private double distance_to_center_of_mass;
    private double mass;
    private double torque_angle_radians;
    private double length;
    private double target;

    public Arm(Motor motor, Arm subArm, Arm parentArm, double mass, double distance, double K_i, double K_d, double K_p)
    {
        pid = new PID(motor.encoder, 0, 0,0 );
        this.motor = motor;
    }


    public double calculate_torque_compensation()
    {
        double sum = this.sum_torque(0 , 0, new double[1], new double[1]);
        double[] point = this.getCenterOfMass(new double[1], new double[1], new double[1], new double[1], 0);
        double compensationPower = (sum * this.motor.free_rpm * Motor.NEWTON_METER_POWER_CONSERVION)/this.motor.max_power;
        return compensationPower;
    };

    //make sure when calling this method sum is zero
    public double sum_torque(double sum, int depth, double[] angles, double[] distances)
    {
        depth++;
        double armAngle = this.getAngle();
        if (angles.length < depth) angles = doubleArrayExpand(angles, angles.length * 2);
        if (distances.length < depth) angles = doubleArrayExpand(angles, angles.length * 2);
        angles[depth - 1] = armAngle;

        double calculationAngle = 0;
        double totalDistance = 0;
        for (double distance: distances)
        {
            totalDistance += distance;
        }

        for (double angle : angles)
        {
            calculationAngle += angle;
        }

        if (depth % 2 != 0) calculationAngle = 180 - calculationAngle; //odd call 180 - angle

        calculationAngle = Math.toRadians(90 - calculationAngle);
        distances[depth - 1] = Math.cos(90 + calculationAngle);
        sum += Math.sin(calculationAngle) * (this.distance_to_center_of_mass + totalDistance) * FORCE_GRAVITY; //calculate moment on arm
        if (this.subArm != null) sum += this.subArm.sum_torque(sum, depth, angles, distances); //sum the torque acting on subjoints
        return sum;
    }
    /*
       returns an array of size 2 , contaning 2 points, first the x point, then y point
       of the center of mass of the form the given arm, in the unit the lengths were measured in

    */

    private double[] getCenterOfMass(double[] xVecs, double[] xVecsMass, double[] yVecs, double[] yVecsMass, int depth)
    {
        depth++;
        double angle = Math.toRadians(this.getAngle());
        double xVec = Math.cos(angle) * this.distance_to_center_of_mass;
        double yVec = Math.sin(angle) * this.distance_to_center_of_mass;
        if (xVecs.length < depth) xVecs = doubleArrayExpand(xVecs, xVecs.length * 2);
        if (xVecsMass.length < depth) xVecsMass = doubleArrayExpand(xVecsMass, xVecsMass.length * 2);
        if (yVecsMass.length < depth) yVecsMass = doubleArrayExpand(yVecsMass, yVecsMass.length * 2);
        if (yVecs.length <= depth) yVecs = doubleArrayExpand(yVecs, yVecs.length * 2);
        xVecs[depth - 1] = xVec;
        xVecsMass[depth - 1] = xVec * this.mass;
        yVecs[depth - 1] = yVec;
        yVecsMass[depth - 1] = yVec * this.mass;
        if (this.subArm == null)
        {
            double numerator = 0;
            double denomanator = 0;
            for (double xVecMass: xVecsMass)
            {
                numerator += xVecMass;
            }

            for (double xvec: xVecs)
            {
                denomanator += xvec;
            }

            double xCenter = numerator/denomanator;
            numerator = denomanator = 0;

            for (double yVecMass: yVecsMass)
            {
                numerator += yVecMass;
            }

            for (double yvec: yVecs)
            {
                denomanator += yvec;
            }

            double yCenter = numerator/denomanator;

            double[] point = new double[2];
            point[0] = xCenter;
            point[1] = yCenter;
            return point;
        }
        else return subArm.getCenterOfMass(xVecs, xVecsMass, yVecs, yVecsMass, depth);
    };

    private double[] doubleArrayExpand(double[] array, int newSize)
    {
        if (array.length - 1 >= newSize) return null;
        double[] newArray = new double[newSize];

        for (int index = 0; index < array.length - 1; index++)
        {
            newArray[index] = array[index];
        }
        return newArray;
    }

    public double getAngle()
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