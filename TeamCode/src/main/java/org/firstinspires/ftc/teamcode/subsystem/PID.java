package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class PID
{
    private DcMotor motor;
    private double K_p;
    private double K_d;
    private double K_i;

    public PID(DcMotor motor, double Ki, double Kd, double Kp)
    {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double calcPower()
    {
        double encoderVal = (double) this.motor.getCurrentPosition();
        return encoderVal;
    }

}