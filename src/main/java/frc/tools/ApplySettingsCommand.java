// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.tools;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command that applies a bunch of settings */
public class ApplySettingsCommand extends Command
{
  // Record that holds name and value of a setting
  record Setting(String name, double value)
  {
    public void apply()
    { // Write the value to SmartDashboard
      SmartDashboard.putNumber(name, value);
    }
  }

  private final List<Setting> settings = new ArrayList<>();

  /** @param name Name of the command */
  public ApplySettingsCommand(final String name)
  {
    setName(name);
  }

  /** Add a setting that the command will apply
   *  @param name Name of dashboard setting
   *  @param value Value to write to that setting
   */
  public void add(final String name, final double value)
  { // Add a setting to the list
    settings.add(new Setting(name, value));
  }

  @Override
  public boolean runsWhenDisabled()
  { // Allow this command to run when disabled
    return true;
  }

  @Override
  public void initialize()
  { // Apply all the settings
    for (Setting setting : settings)
      setting.apply();
  }

  @Override
  public boolean isFinished()
  { // This command is immediately finished
    return true;
  }
}
