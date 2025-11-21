// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demo;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.parts.SparkMini;
import frc.tools.CommandRobotBase;

/** SparkMini Motor Demo */
public class MotorSparkMiniDemoRobot extends CommandRobotBase
{
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final SparkMini motor = new SparkMini(1);

    @Override
    public void teleopPeriodic()
    {
        motor.setVoltage(joystick.getRightY()*12.0);
    }
}
