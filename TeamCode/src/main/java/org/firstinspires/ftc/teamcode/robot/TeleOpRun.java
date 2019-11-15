package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Arm;
import org.firstinspires.ftc.teamcode.control.Motor;
import org.firstinspires.ftc.teamcode.subsystem.DriveBase;

@TeleOp(name="test2019", group = "Linear OpMode")

public class TeleOpRun extends LinearOpMode
{
    private Robot robot;

    @Override
    public void runOpMode()
    {
        this.robot = new Robot(hardwareMap);
        waitForStart();
        while (opModeIsActive())
        {
            Time.loopStartTime = System.currentTimeMillis();
            this.robot.driveBase.teleOpdrive(gamepad1);
            this.robot.servo.setPosition(gamepad2.left_stick_y * 180);
            long now = System.currentTimeMillis();
            Time.lastDelta = now - Time.loopStartTime;
            Time.averageDelta += Time.lastDelta;
            Time.averageDelta /= 2;
        }

    }
}
