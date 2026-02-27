// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.tools.CommandRobotBase;

/** Storage test */
public class StorageTestRobot extends CommandRobotBase
{
    private final Storage storage = new Storage();

    @Override
    public void teleopPeriodic()
    {
        storage.run(RobotOI.joystick.a().getAsBoolean());
    }
}
