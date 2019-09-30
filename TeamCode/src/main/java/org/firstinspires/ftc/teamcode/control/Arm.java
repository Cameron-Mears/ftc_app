package org.firstinspires.ftc.teamcode.control;


public class Arm
{
    private static final double FORCE_GRAVITY = 9.8;

    private Arm subArm; //if there is another arm joined.
    private Motor motor;
    private double distance_to_center_of_mass;
    private double mass;
    private double torque_angle_radians;
    private double target;

    public Arm()
    {

    }


    public double calculate_torque_compenstion()
    {
        double sum = 0;
        sum = this.sum_torque(sum);
        double compensationPower = (sum * this.motor.free_rpm * Motor.NEWTON_METER_POWER_CONSERVION)/this.motor.max_power;
        return compensationPower;
    };

    //make sure when calling this method sum is zero
    public double sum_torque(double sum)
    {
        if (this.subArm != null) sum += this.subArm.sum_torque(sum); //sum the torque acting on subjoints
        sum += Math.sin(this.torque_angle_radians) * this.distance_to_center_of_mass * FORCE_GRAVITY; //calculate moment on arm
        return sum;
    }

}