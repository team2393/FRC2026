// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.swervelib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Forward/backwards part of swerve module
 *
 *  Controls motor voltage using feed-forward
 *  based on static voltage (ks) and velocity gain (ks),
 *  plus proportional correction
 */
abstract public class DriverBase extends SubsystemBase
{
  //Network table entries
  private final NetworkTableEntry nt_position;
  private final NetworkTableEntry nt_speed;
  private final NetworkTableEntry nt_speed_sp;
  private final NetworkTableEntry nt_ks;
  private final NetworkTableEntry nt_kv;
  private final NetworkTableEntry nt_P;
  private final NetworkTableEntry nt_I;
  private final NetworkTableEntry nt_D;

  // Information specific to each driver
  private double zero_position = 0.0;
  private double simulated_speed = 0.0;
  private double simulated_position = 0.0;
  private PIDController pid = new PIDController(0, 0, 0);

  /** Construct Driver
   *  @param index Driver index 0..3
   *  @param ks Static voltage
   *  @param kv Velocity gain
   *  @param P Proportional gain
   *  @param I Integral gain
   *  @param D Differential gain
   */
  public DriverBase(int index, double ks, double kv, double P, double I, double D)
  {
    // Position and speed are specific to driver
    nt_position = SmartDashboard.getEntry("Position" + index);
    nt_speed = SmartDashboard.getEntry("Speed" + index);
    nt_speed_sp = SmartDashboard.getEntry("SpeedSetp" + index);

    // Feed-forward and PID settings are the same for all drivers
    nt_ks = SmartDashboard.getEntry("Driver ks");
    nt_kv = SmartDashboard.getEntry("Driver kv");
    nt_P = SmartDashboard.getEntry("Driver P");
    nt_I = SmartDashboard.getEntry("Driver I");
    nt_D = SmartDashboard.getEntry("Driver D");
    nt_ks.setDefaultDouble(ks);
    nt_kv.setDefaultDouble(kv);
    nt_P.setDefaultDouble(P);
    nt_I.setDefaultDouble(I);
    nt_D.setDefaultDouble(D);

      // Defaults: 1, Double.POSITIVE_INFINITY
      pid.setIntegratorRange(-10, 10);
      pid.setIZone(0.1);    
  }

  /** @param name Name under which to publish PID on dashboard */
  public void publishPID(String name)
  {
    SmartDashboard.putData(name, pid);
  }

  /** Reset position to zero */
  public void resetPosition()
  {
    pid.reset();
    zero_position = getRawPosition();
    simulated_position = 0.0;
  }

  /** @return Get position in meters (without zero offset) */
  abstract protected double getRawPosition();

  /** @return Get speed in meters/sec (won't be called in simulation) */
  abstract protected double getRealSpeed();

  /** @return Get speed in meters/sec */
  public double getSpeed()
  {
    if (RobotBase.isSimulation())
      return simulated_speed;
    return getRealSpeed();
  }

  /** @param voltage Voltage to motor for driving the swerve module */
  abstract public void setVoltage(double voltage);

  /** @return Get position in meters from last 'reset' */
  public double getPosition()
  {
    if (RobotBase.isSimulation())
      return simulated_position;
    return getRawPosition() - zero_position;
  }

  /** @param desired_speed Speed in m/s */
  public void setSpeed(double desired_speed)
  {
    nt_speed_sp.setDouble(desired_speed);
    double feed_forward = nt_ks.getDouble(0.0) * Math.signum(desired_speed) + nt_kv.getDouble(0.0) * desired_speed;
    pid.setPID(nt_P.getDouble(0.0), nt_I.getDouble(0.0), nt_D.getDouble(0.0));
    double prop_correction =  pid.calculate(getSpeed(), desired_speed);
    setVoltage(feed_forward + prop_correction);

    // Update simulation, assume being called each period
    simulated_speed = desired_speed;
    simulated_position += desired_speed * TimedRobot.kDefaultPeriod;
  }

  @Override
  public void periodic()
  {
    nt_position.setNumber(getPosition());
    nt_speed.setNumber(getSpeed());
  }
}
