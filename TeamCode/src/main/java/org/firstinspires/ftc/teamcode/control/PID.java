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
    private int lastPos;



    public PID(Motor.Encoder encoder, double Ki, double Kd, double Kp)
    {
        this.encoder = encoder;
        this.integral = 0;

    }

    public double calculate()
    {
        int currentPos = this.encoder.getPosition();
        this.encoder.update();
        double distance = this.targetPos - currentPos;
        integral += (currentPos - lastPos)/2;
        this.derivative = this.encoder.getRate();
        double power = 0;
        power += distance * this.K_p;
        power += this.derivative * this.K_d;
        power += this.integral * this.K_i;
        this.lastPos = currentPos;
        return power;
    }

    public void setTargetPos(int pos)
    {
        this.targetPos = pos;
    }
}