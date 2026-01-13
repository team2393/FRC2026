// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervebot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command for rotating to target using 2D info from camera */
public class RotateToTarget extends Command
{
    PhotonCamera camera = new PhotonCamera("HD_Pro_Webcam_C920");
    private final SwervebotDrivetrain drivetrain;
    private final PIDController pid = new PIDController(2.0, 1.0, 0);

    public RotateToTarget(SwervebotDrivetrain drivetrain)
    {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        SmartDashboard.putData("rotate_to_target", pid);
    }

    @Override
    public void execute()
    {
        double target_angle = Double.NaN;
        for (var result : camera.getAllUnreadResults())
            if (result.hasTargets())
                for (var target : result.getTargets())
                {
                    // XXX Check target ID, use only 'centered' targets?
                    // XXX Also use target.getArea() to estimate distance?
                    target_angle = target.getYaw();
                    // System.out.println("Tag " + target.getFiducialId() + ": yaw " + target_angle);
                }

        if (Double.isNaN(target_angle))
        {
            // No camera reading.
            // Stop drivetrain? That results in stutter.
            // Keep going? Should have a timer to stop when no update for 1-2 seconds...
            return;
        }

        // Rotation in deg/sec, limited
        double rotate = pid.calculate(target_angle);
        rotate = MathUtil.clamp(rotate, -SwervebotDrivetrain.MAX_ROTATION_DEG_PER_SEC,
                                        +SwervebotDrivetrain.MAX_ROTATION_DEG_PER_SEC);
        // Convert to rad/sec
        drivetrain.swerve(0, 0, Math.toRadians(rotate));
    }
}
