package org.firstinspires.ftc.teamcode.physicscalculations;

import org.firstinspires.ftc.teamcode.control.Arm;

public final class PhysicsCalculations
{
    /*
       Calculates the new point position of a point after it has been rotated angle
       degrees. sin and cos are saved as are relatively costly to compute and called
       many times, with same values
     */
    public static PointMass[] rotationalTransformPoints(PointMass[] points, double angle)
    {
        double cos = Math.cos(Math.toRadians(angle));
        double sin = Math.sin(Math.toRadians(angle));

        PointMass[] pointsTransformed = new PointMass[points.length];
        for (int index = 0; index < points.length - 1; index ++)
        {
            PointMass point = points[index];
            double x = point.getX();
            double y = point.getY();
            double newX = (x * cos) + (y * sin);
            double newY = (y * cos) - (x * sin);
            pointsTransformed[index] = new PointMass(newX, newY, point.getMass());
        }


        return pointsTransformed;

    }

    /*
        compute moment of inertia on arm, so the program knows how much power
        to put to motor to get desired angular acceleration
     */
    public double computeMomentOfInertia(double baseX, double baseY, double sum, double angles, Arm arm)
    {
        angles += arm.computeAngle();
        double angle = Math.toRadians(angles);
        double distance, cos, sin;
        cos = Math.cos(angle);
        sin = Math.sin(angle);
        PointMass[] transformed = PhysicsCalculations.rotationalTransformPoints(arm.attachments, angles);
        for (PointMass pointMass: transformed)
        {
            distance = Math.hypot( baseX + pointMass.getX(), baseY + pointMass.getY());
            sum += distance * distance * pointMass.getMass();
        }
        distance =  Math.hypot( baseX, baseY);
        sum += (1/3) * (arm.length + distance) * (arm.length + distance) * arm.massOfBeam;

        baseX += Math.cos(angle) * arm.length;
        baseY += Math.sin(angle) * arm.length;
        if (arm.subArm == null) return sum;
        else return computeMomentOfInertia( baseX, baseY, sum, angles, arm.subArm);
    }


}
