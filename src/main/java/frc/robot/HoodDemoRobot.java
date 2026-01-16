// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
        hood.holdPosition();
    }
}