package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Arm;
import org.firstinspires.ftc.teamcode.control.Motor;
import org.firstinspires.ftc.teamcode.subsystem.DriveBase;

@TeleOp(name="test", group = "Linear OpMode")

public class Robot extends LinearOpMode
{

    private DriveBase driveBase;

    private void Robot_init()
    {
        this.driveBase = new DriveBase(new Motor(hardwareMap.dcMotor.get("up"), true), new Motor(hardwareMap.dcMotor.get("down"), true), new Motor(hardwareMap.dcMotor.get("left"), true), new Motor(hardwareMap.dcMotor.get("right"), true));

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        this.Robot_init();
        waitForStart();

        while (opModeIsActive())
        {
            this.driveBase.teleOpdrive(gamepad1);
        }

    }
}