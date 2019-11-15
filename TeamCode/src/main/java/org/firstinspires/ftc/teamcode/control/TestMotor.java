package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TestMotor
{
    public static final double NEWTON_METER_POWER_CONSERVION = 0.1047;
    public int encoderPlusesPerRevolution;
    public int gearRatio;
    public DcMotor motor;
    public Encoder encoder;
    public double free_rpm;
    public double max_power;

    public TestMotor(DcMotor motor, boolean createEncoder)
    {
        this.motor = motor;
        if (createEncoder) this.encoder = new Encoder(motor, 0);
    }

    public class Encoder
    {
        private DcMotor motor;
        private double rate;
        private int position;
        private long lastTime;
        private double deltaT;

        public Encoder(DcMotor motor, int startPos)
        {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.position = startPos;
            this.motor = motor;
            this.rate = 0;
            this.lastTime = System.currentTimeMillis();
        }

        public void update()
        {
            long now = System.currentTimeMillis();
            this.deltaT = (double) (now - this.lastTime);
            this.lastTime = now;
            int newPos = this.motor.getCurrentPosition();
            this.rate = (newPos - this.position)/this.deltaT;
            this.position = newPos;
        }

        public int getPosition()
        {
            return this.position;
        }

        public double getRate()
        {
            return this.rate;
        }

        public double getDeltaT()
        {
            return this.deltaT;
        }

    }


}