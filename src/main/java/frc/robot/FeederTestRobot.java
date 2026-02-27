// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.swervelib.SwerveOI;
import frc.tools.CommandRobotBase;

/** Feeder test */
public class FeederTestRobot extends CommandRobotBase
{
    private Feeder feeder = new Feeder();

    @Override
    public void teleopPeriodic()
    {
        feeder.run(SwerveOI.joystick.y().getAsBoolean()
                   ? Feeder.Mode.SHOOT
                   : Feeder.Mode.OFF);
    }

    @Override
    public void autonomousPeriodic()
    {
        feeder.run(Feeder.Mode.FEED);
    }
}
