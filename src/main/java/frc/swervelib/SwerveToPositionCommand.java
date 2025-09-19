// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervelib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command for swerving to a position */
public class SwerveToPositionCommand extends Command
{
  /** Proportional gain for distance control */
  private static final NetworkTableEntry nt_stp_P = SmartDashboard.getEntry("SwerveToPos_P");
  static
  {
    nt_stp_P.setDefaultDouble(3.0);
  }

  /** Max. speed */
  public static double MAX_SPEED = 2.0;

  /** Max. acceleration */
  public static double ACCEL = 3.0;

  private final SwerveDrivetrain drivetrain;
  private final double x, y, heading;
  private double last_speed, distance, angle;

  /** @param drivetrain
   *  @param x Desired X position
   *  @param y Desired Y position
   */
  public SwerveToPositionCommand(SwerveDrivetrain drivetrain, double x, double y)
  {
    this(drivetrain, x, y, Double.NaN);
  }

  /** @param drivetrain
   *  @param x Desired X position
   *  @param y Desired Y position
   *  @param heading Desired heading
   */
  public SwerveToPositionCommand(SwerveDrivetrain drivetrain, double x, double y, double heading)
  {
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.heading = heading;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    last_speed = 0.0;
    distance = 20.0;
    angle = 0.0;
  }

  // We generally rely on the trajectory tools to follow arbitrary paths.
  // In this case, 'initialize' could create a simple 2-point trajectory
  // from drivetrain.getPose() to the desired (x, y) and then
  // launch a drivetrain.followTrajectory(...).
  // However, such a trajectory computation fails when the distance
  // between the points is too small (~10 cm), and we want to support
  // even small moves with this command.
  public void execute()
  {
    // Compute direct line from where we are right now to the desired location
    Pose2d pose = drivetrain.getPose();
    double dx = x - pose.getX();
    double dy = y - pose.getY();

    // Distance, angle relative to the current robot heading
    distance = Math.hypot(dx, dy);
    angle = Math.toDegrees(Math.atan2(dy, dx)) - pose.getRotation().getDegrees();

    // Proportional control of speed based on distance
    double speed = Math.min(MAX_SPEED, nt_stp_P.getDouble(0)*distance);

    // Limit acceleration
    if (speed > last_speed)
      speed = Math.min(speed, last_speed + ACCEL * TimedRobot.kDefaultPeriod);
    last_speed = speed;

    // Should we also rotate to a desired heading?
    double rotation;
    if (Double.isNaN(heading))
      rotation = 0.0;
    else
    {
      // Proportional control of rotational speed based on angle
      double offset = Math.IEEEremainder(heading - drivetrain.getPose().getRotation().getDegrees(), 360.0);
      rotation = MathUtil.clamp(RotateToHeadingCommand.P*offset,
                                -RotateToHeadingCommand.MAX_SPEED,
                                RotateToHeadingCommand.MAX_SPEED);
    }
    drivetrain.swerve(speed*Math.cos(Math.toRadians(angle)),
                      speed*Math.sin(Math.toRadians(angle)),
                      Math.toRadians(rotation));
  }

  @Override
  public boolean isFinished()
  {
    // Within 0.5 cm?
    return distance < 0.005;
  }

  @Override
  public void end(boolean interrupted)
  {
    drivetrain.stop();
  }
}
