// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;

/** Test the intake */
public class IntakeTestRobot extends CommandRobotBase
{
    private final PowerDistribution power_dist = new PowerDistribution();
    private final Intake intake = new Intake();

    public IntakeTestRobot()
    {
        power_dist.clearStickyFaults();
        SmartDashboard.putData("Power", power_dist);
    }

    @Override
    public void teleopPeriodic()
    {
        intake.open(RobotOI.joystick.a().getAsBoolean());
    }

    @Override
    public void autonomousPeriodic()
    {
        intake.open(true);
    }
}
