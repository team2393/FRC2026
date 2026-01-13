// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.swervelib.SwerveDrivetrain;

/** Command that aims at hub
 *
 *  Aims robot at hub based on odometry,
 *  which is ideally updated from camera info.
 */
public class AimToHub extends Command
{
    // TODO Update with actual hub positions
    private static final Translation2d BLUE_HUB = new Translation2d(4.49, 4.041);
    private static final Translation2d RED_HUB = new Translation2d(13.04, 4.041);

    private final SwerveDrivetrain drivetrain;
    private Translation2d hub = null;
    private double angle_error = 0.0;

    public AimToHub(SwerveDrivetrain drivetrain)
    {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {
        // Use "our" hub.
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            hub = RED_HUB;
        else
            hub = BLUE_HUB;
    }

    @Override
    public void execute()
    {
        // Where are we?
        Pose2d robot_pose = drivetrain.getPose();

        // Direction from there to hub
        Translation2d direction = hub.minus(robot_pose.getTranslation());
        Rotation2d angle = direction.getAngle();
        // double distance = direction.getNorm();

        // Proportional control of robot heading with limit
        angle_error = angle.minus(robot_pose.getRotation()).getDegrees();
        // System.out.println("Direction " + direction + ", angle " + angle + ", error " + angle_error);
        double vr = 6.0 * angle_error;
        vr = MathUtil.clamp(vr, -180, +180);
        drivetrain.swerve(0, 0, Math.toRadians(vr));

        // XXX Could use distance to control spinner speed
    }

    @Override
    public boolean isFinished()
    {
        // We're done once the error is small enough
        return Math.abs(angle_error) < 1.0;
    }
}
