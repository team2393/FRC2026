// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.tools.CommandRobotBase;

/** Hood test robot
 *
 *  [ ] In teleop, calibrate STEPS_PER_PERC,
 *      check direction (right stick forward == up)
 *
 *  [ ] In auto, configure Hood PID
 */
public class HoodDemoRobot extends CommandRobotBase
{
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Hood hood = new Hood();
    private final PowerDistribution power_dist = new PowerDistribution();

    public HoodDemoRobot()
    {
        // Power dist. info
        power_dist.clearStickyFaults();
        SmartDashboard.putData("Power", power_dist);
    }

    @Override
    public void disabledInit()
    {
        // 0 or less disables the hood position hold
        SmartDashboard.putNumber("HoodSetpoint", -1);
    }

    @Override
    public void teleopInit()
    {
        hood.reset();
        SmartDashboard.putNumber("HoodSetpoint", -1);
    }

    @Override
    public void teleopPeriodic()
    {
        // Disable the 'holdPosition()' call in Hood::periodic()!
        // Pushing forward/up sends positive voltage
        hood.setVoltage(-12.0*joystick.getRightY());
    }

    @Override
    public void autonomousPeriodic()
    {
        double setpoint = ((System.currentTimeMillis() / (int)(SmartDashboard.getNumber("Period", 5.0)*1000)) % 2 == 1)
                        ? 20.0 : 80.0;
        SmartDashboard.putNumber("HoodSetpoint", setpoint);
    }
}