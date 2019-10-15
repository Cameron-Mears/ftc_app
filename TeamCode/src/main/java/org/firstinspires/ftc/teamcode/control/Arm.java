package org.firstinspires.ftc.teamcode.control;


public class Arm
{
    private static final double FORCE_GRAVITY = 9.8;

    private Arm subArm; //if there is another arm joined
    private Arm parentArm;
    private PID pid;
    private Motor motor;
    private double distance_to_center_of_mass;
    private double mass;
    private double torque_angle_radians;
    private double target;

    public Arm(Motor motor, Arm subArm, Arm parentArm, double mass, double distance, double K_i, double K_d, double K_p)
    {
        pid = new PID(motor.encoder, 0, 0,0 );
        this.motor = motor;
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
        sum += Math.sin(this.getTorque_angle_radians()) * this.distance_to_center_of_mass * FORCE_GRAVITY; //calculate moment on arm
        return sum;
    }

    public double getTorque_angle_radians()
    {
        this.motor.encoder.update();
        double encoderPosition = motor.encoder.getPosition();
        double angleDegrees = ((encoderPosition % this.motor.encoderPlusesPerRevolution)/this.motor.encoderPlusesPerRevolution) * 360;
        return ((angleDegrees) * (Math.PI/180));
    }

    public void setTargetAngle(double angleDegrees)
    {
        this.pid.setTargetPos((int)(angleDegrees/360 * this.motor.encoderPlusesPerRevolution));
    }

    public void update()
    {
        this.motor.motor.setPower(this.calculate_torque_compenstion() + this.pid.calculate());
    }



}