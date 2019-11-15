package org.firstinspires.ftc.teamcode.util;

public class KeyInUseException extends Exception
{
    public String key;

    public KeyInUseException(String key)
    {
        this.key = key;
    }
}
