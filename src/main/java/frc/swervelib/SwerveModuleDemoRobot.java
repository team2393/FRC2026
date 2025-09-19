// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.swervelib;

import frc.tools.CommandRobotBase;

/** Robot for testing one or more swerve modules */
public class SwerveModuleDemoRobot extends CommandRobotBase
{
  private final SwerveModule modules[];

  public SwerveModuleDemoRobot(SwerveModule... modules)
  {
    this.modules = modules;
    SwerveOI.reset();
  }

  @Override
  public void teleopPeriodic()
  {
    if (SwerveOI.resetDrivetrain().getAsBoolean())
      for (SwerveModule module : modules)
        module.resetPosition();

    double vx = SwerveOI.getForwardSpeed();
    double vy = SwerveOI.getLeftSpeed();
    double speed = Math.sqrt(vx*vx + vy*vy);
    double angle = Math.toDegrees(Math.atan2(vy, vx));
    for (SwerveModule module : modules)
      module.drive(angle, speed);
  }
}
