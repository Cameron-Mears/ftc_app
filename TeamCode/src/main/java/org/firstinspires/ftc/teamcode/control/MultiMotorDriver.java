package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.hardware.ExpansionHubMotor;

/*
    Put some number of motors as arg, rest of the system will treat it as one motor

    if any motors need to be run backwards list powersign as -1, otherwise set as 1

    Make sure all motors all same gearing, or else it be messed up,
    get use the same motor type.
 */
public class MultiMotorDriver implements Motor, CurrentTorqueRelation
{

    private ExpansionHubMotor[] drivers;
    private int[] powerSigns;
    private MotorProperties properties;

    private int encodingMotor;
    private double rate;
    private long lastTime;
    private int position;
    private int lastPosition;
    private int target;

    public MultiMotorDriver(MotorProperties motorProperties, int encodingMotor, int[] powerSigns, ExpansionHubMotor ... motors)
    {
        assert (powerSigns.length == motors.length);

        this.powerSigns = powerSigns;
        this.properties = motorProperties;
        this.encodingMotor = encodingMotor;
        this.drivers = motors;
        this.lastTime = System.currentTimeMillis();
        this.lastPosition = motors[encodingMotor].getCurrentPosition();

    }

    @Override
    public double currentDrawAmperes()
    {
        return 0;
    }

    @Override
    public CurrentTorqueRelation getCurrentTorqueRelation()
    {
        return null;
    }

    @Override
    public void setTargetPosition(double position)
    {
        target = (int)position;
    }

    @Override
    public double getRate() //returns raw rate, of the encoding motor in ticks/second
    {
        return rate;
    }


    @Override
    public void update()
    {
        DcMotor motor = drivers[encodingMotor];
        long delta = System.currentTimeMillis() - lastTime;
        int currentPos = motor.getCurrentPosition();
        this.rate = (currentPos - lastPosition)/delta;
        lastTime = System.currentTimeMillis();
        position = currentPos;
        lastPosition = position;

    }

    @Override
    public int getPosition()
    {
        return position;
    }

    @Override
    public void setPower(double power)
    {
        for (int index = 0; index < drivers.length; index ++)
        {
            DcMotor motor = drivers[index];
            motor.setPower(power * powerSigns[index]);
        }
    }

    @Override
    public MotorProperties getProperties()
    {
        double ampDraw = drivers[0].getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
        return properties;
    }

    @Override
    public void toTorque(double amperage)
    {

    }

    @Override
    public void toAmperage(double torque)
    {

    }
}
