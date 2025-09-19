// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.tools;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command that applies an adjustable setting
 * 
 *  Example:
 *  <pre>
 *  new ApplyAdjustableSettingCommand("Lift Mid", "Lift Mid Setpoint",  0.6, "Lift Setpoint")
 *  </pre>
 *  creates a command "Lift Mid" that can be placed on dashboard or bound to <code>button.onTrue()</code>.
 *  Creates "Lift Mid Setpoint" for adjustment on dashboard with initial value 0.6.
 *  When invoked the command writes the current value to "Lift Setpoint".
 */
public class ApplyAdjustableSettingCommand extends Command
{
  private NetworkTableEntry nt_value, nt_setting;

  /**@param command_name Name of this command like "Move Lift Low" will show on dashboard
   * @param value_name Name of the adjustable value like "Lift Low Setpoint" for dashboard
   * @param initial Initial value of the "Lift Low Setpoint"
   * @param setting Name of setting that's updated when command runs, like "Lift Setpoint"
   */
  public ApplyAdjustableSettingCommand(final String command_name,
                                       final String value_name,
                                       final double initial,
                                       final String setting)
  {
    setName(command_name);
    nt_value = SmartDashboard.getEntry(value_name);
    nt_value.setDefaultDouble(initial);
    nt_setting = SmartDashboard.getEntry(setting);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  @Override
  public void initialize()
  {
    nt_setting.setNumber(nt_value.getDouble(0.0));
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }
}
