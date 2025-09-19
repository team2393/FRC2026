// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command for human to drive robot relative to field odometry
 *
 *  Assuming correct field odometry, "forward" will drive away from the drive station
 */
public class AbsoluteSwerveCommand extends Command
{
  private final SwerveDrivetrain drivetrain;
  private double last_heading;

  /** @param drivetrain */
  public AbsoluteSwerveCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    last_heading = drivetrain.getPose().getRotation().getDegrees();
  }

  public void execute()
  {
    // Speed vector (vx, vy) meant to be in field coordinates,
    // vx going "up" from the origin of the field along X
    // For the blue side, this means "forward" is away from the
    // driver station and "right" is to the right
    double vx, vy;
    int pov = SwerveOI.joystick.getHID().getPOV();
    if (pov >= 0)
    {
      double angle = Math.toRadians(-pov);
      vx = SwerveDrivetrain.CRAWL_SPEED * Math.cos(angle);
      vy = SwerveDrivetrain.CRAWL_SPEED * Math.sin(angle);
    }
    else
    {
      vx = SwerveOI.getForwardSpeed();
      vy = SwerveOI.getLeftSpeed();
    }

    // When at the red side of the field,
    // rotate by 180 deg so that "forward" is again
    // away from the driver and "right" moves to the right.
    // In simulation GUI select alliance from DS/FMS
    // while Robot State is "Disconnected"
    if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
    {
      vx = -vx;
      vy = -vy;
    }
    // If robot points in the field "x" direction, we could use (vx, vy) as given,
    // but we generally need to rotate (vx, vy) backwards from the current heading
    // of the robot to see how the robot needs to move:
    // double heading = drivetrain.getHeading().getDegrees();
    double heading = drivetrain.getPose().getRotation().getDegrees();
    double correction = -heading;

    // Correct the correction by fraction of angular speed,
    // https://www.chiefdelphi.com/t/field-relative-swervedrive-drift-even-with-simulated-perfect-modules
    // First determine change in heading _per_second_, assuming we run at 50 Hz
    double rotational_speed = (heading - last_heading) / 0.02;
    SmartDashboard.putNumber("HeadingChange", rotational_speed);
    last_heading = heading;
    // Basic idea:
    // While rotating,the drivetrain heading is a little old,
    // so guess what it's now based on rotational speed.
    // 0.15 is a guess from the posting mentioned above
    correction -= 0.15 * rotational_speed;
    Translation2d absoluteDirection = new Translation2d(vx, vy).rotateBy(Rotation2d.fromDegrees(correction));

    // Swerve robot in 'absoluteDirection', while rotating as requested
    double vr = Math.toRadians(SwerveOI.getRotationSpeed());
    drivetrain.swerve(absoluteDirection.getX(), absoluteDirection.getY(), vr, SwerveDrivetrain.CENTER);
  }

  @Override
  public void end(boolean interrupted)
  {
    drivetrain.stop();
  }
}
