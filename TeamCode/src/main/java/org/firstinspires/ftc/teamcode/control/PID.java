package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Motor;


public class PID
{
    private Motor.Encoder encoder;
    private double K_p;
    private double K_d;
    private double K_i;
    private double integral;
    private double derivative;

    private int targetPos;



    public PID(Motor.Encoder encoder, double Ki, double Kd, double Kp)
    {
        this.encoder = encoder;
        this.integral = 0;

    }

    public double calculate()
    {
        this.encoder.update();
        double distance = this.targetPos - this.encoder.getPosition();
        this.derivative = this.encoder.getRate();
        double power = 0;
        power += distance * this.K_p;
        power += this.derivative * this.K_d;
        return power;
    }
}