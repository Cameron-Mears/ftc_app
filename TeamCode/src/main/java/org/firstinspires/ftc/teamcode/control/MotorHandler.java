package org.firstinspires.ftc.teamcode.control;


import org.firstinspires.ftc.teamcode.util.HashTable;
import org.firstinspires.ftc.teamcode.util.KeyInUseException;

import java.util.LinkedList;

public class MotorHandler
{

    private HashTable<String, Motor> table;
    private LinkedList<Motor> motors;

    public boolean add(String name, Motor motor) throws KeyInUseException
    {
        return table.put(name,motor) && motors.add(motor);

    }

    public void update()
    {
        for (Motor motor : motors)
        {
            motor.update();
        }
    }

    public Motor getByName(String name)
    {
        return table.get(name);
    }
    
}
