// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demo;

import frc.tools.CommandRobotBase;

/** This robot communicates with the field control system.
 *  It would be allowed to participate in matches,
 *  and might be the perfect first defense bot
 */
public class FirstRobotDemo extends CommandRobotBase
{
    @Override
    public void disabledInit()
    {
        System.out.println("I'm doing nothing");
    }

    @Override
    public void teleopInit()
    {
        System.out.println("I'm starting tele-op!");
    }

    @Override
    public void autonomousInit()
    {
        System.out.println("I'm doing something autonomously...");
    }
}
