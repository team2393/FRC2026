// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to wait for stable position
 *
 *  Assuming camera is updating position,
 *  wait until it doesn't change any more
 *
 *  Only works when robot is not driving!
 */
public class WaitForStablePosition extends Command
{
  private final SwerveDrivetrain drivetrain;
  private final NetworkTableEntry camera_status;
  private final double min_stable_secs;
  private final double min_distance;
  // How long have we been stable?
  private final Timer timer = new Timer();
  // Last position, to compare against current position
  private Pose2d last_position;

  /** @param drivetrain ..to monitor
   *  @param camera_status Does camera have information?
   *  @param min_stable_secs How long info should be stable
   *  @param min_distance Distance in meters that's considered stable
   */
  public WaitForStablePosition(SwerveDrivetrain drivetrain, String camera_status, double min_stable_secs, double min_distance)
  {
    this.drivetrain = drivetrain;
    this.camera_status = SmartDashboard.getEntry(camera_status);
    this.min_stable_secs = min_stable_secs;
    this.min_distance = min_distance;
    // DON'T require drivetrain because we're only reading, not controlling it
    // addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    timer.restart();
    last_position = drivetrain.getPose();
  }

  // Very rough estimate of robot size to check for movement
  final static private double RADIUS = 0.5;

  @Override
  public boolean isFinished()
  { // In simulation, we're fine after min stable secs
    if (RobotBase.isSimulation())
      return timer.hasElapsed(min_stable_secs);

    Pose2d position = drivetrain.getPose();

    if (!camera_status.getBoolean(false))
    { // No camera info, can't be stable
      timer.restart();
      last_position = position;
      return false;
    }

    // How much did we move?
    Transform2d diff = position.minus(last_position);
    // Consider swerve distance as well as rotation,
    // using length of traveled arc
    double distance = diff.getTranslation().getNorm() +
                      Math.abs(diff.getRotation().getRadians() * RADIUS);
    System.out.println("WaitForStablePosition distance: " + distance);
    last_position = position;

    if (distance > min_distance)
    { // Not stable
      timer.restart();
      return false;
    }

    // Have we been stable for the requested time?
    return timer.hasElapsed(min_stable_secs);
  }
}
