// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervelib;

import edu.wpi.first.wpilibj2.command.Command;

/** Command for locking the drivetrain */
public class LockDrivetrainCommand extends Command
{
  private final SwerveDrivetrain drivetrain;

  public LockDrivetrainCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public void execute()
  {
    drivetrain.lock();
  }

  @Override
  public void end(boolean interrupted)
  {
    drivetrain.stop();
  }
}
