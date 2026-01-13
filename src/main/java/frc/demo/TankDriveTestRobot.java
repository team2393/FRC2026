// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.demo;

import frc.robot.RobotOI;
import frc.tools.CommandRobotBase;

/** Tank test robot */
public class TankDriveTestRobot extends CommandRobotBase
{
    private final TankDrivetrain drivetrain = new TankDrivetrain();

    @Override
    public void teleopPeriodic()
    {
        drivetrain.drive(RobotOI.getForwardSpeed(), RobotOI.getRotationSpeed());
    }
}
