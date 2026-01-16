// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.tools.CommandRobotBase;

/** Hood test robot
 *
 *  [ ] In teleop, calibrate STEPS_PER_PERC,
 *      check direction (forward == out)
 *
 *  [ ] In auto, configure Hood PID
 */
public class HoodDemoRobot extends CommandRobotBase
{
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Hood hood = new Hood();

    @Override
    public void teleopInit()
    {
        hood.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        hood.setVoltage(0.1*joystick.getRightY());
    }

    @Override
    public void autonomousPeriodic()
    {
        double setpoint = ((System.currentTimeMillis() / (int)(SmartDashboard.getNumber("Period", 5.0)*1000)) % 2 == 1)
                        ? 20.0 : 80.0;
        SmartDashboard.putNumber("Hood", setpoint);
        hood.holdPosition();
    }
}