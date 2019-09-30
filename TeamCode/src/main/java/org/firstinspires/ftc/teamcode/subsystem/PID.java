package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Motor;


public class PID {
    private Motor motor;
    private double K_p;
    private double K_d;
    private double K_i;

    public PID(DcMotor motor, double Ki, double Kd, double Kp) {
        this.motor.motor = motor;
        this.motor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double calcPower() {
        double encoderVal = (double) this.motor.motor.getCurrentPosition();
        return encoderVal;
    }
}