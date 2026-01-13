// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.tools.CommandRobotBase;
import frc.swervelib.RelativeSwerveCommand;

/** FRC2026 robot */
public class Robot extends CommandRobotBase
{
    private final RobotDrivetrain drivetrain = new RobotDrivetrain();
    private final Command joydrive = new RelativeSwerveCommand(drivetrain);

    @Override
    public void teleopInit()
    {
        CommandScheduler.getInstance().schedule(joydrive);
    }
}
