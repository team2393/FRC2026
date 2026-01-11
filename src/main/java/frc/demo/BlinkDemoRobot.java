// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demo;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.tools.CommandRobotBase;

/** LED blink demo */
public class BlinkDemoRobot extends CommandRobotBase
{
    private final CommandXboxController joystick = new CommandXboxController(0);
    private DigitalOutput led = new DigitalOutput(7);

    @Override
    public void teleopPeriodic()
    {
        // Turn LED on/off with button A
        led.set(joystick.a().getAsBoolean());
    }

    @Override
    public void autonomousPeriodic()
    {
        // Change every 500 ms
        boolean on = (System.currentTimeMillis() / 500) % 2 == 1;
        led.set(on);
    }
}
