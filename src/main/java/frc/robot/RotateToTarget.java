// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.swervelib.SwerveDrivetrain;

/** Command for rotating to target using 2D info from camera */
public class RotateToTarget extends Command
{
    private final PhotonCamera camera = new PhotonCamera("Front");
    private final SwerveDrivetrain drivetrain;
    private final PIDController pid = new PIDController(2.0, 1.0, 0);
    private final Timer timeout = new Timer();

    public RotateToTarget(SwerveDrivetrain drivetrain)
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
                    // Check target ID, use only 'centered' targets
                    int id = target.getFiducialId();
                    if (id != 17)
                        continue;
                    // XXX Also use target.getArea() to estimate distance?
                    target_angle = target.getYaw();
                    // System.out.println("Tag " + target.getFiducialId() + ": yaw " + target_angle);
                    timeout.stop();
                    break;
                }

        if (Double.isNaN(target_angle))
        {
            // No useful camera reading. Keep going with last 'drive',
            // but time out and then stop if there's no updates
            if (!timeout.isRunning())
                timeout.start();
            else if (timeout.hasElapsed(1.0))
                drivetrain.stop();
            return;
        }

        // Rotation in deg/sec, limited
        double rotate = pid.calculate(target_angle);
        rotate = MathUtil.clamp(rotate, -SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC,
                                        +SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC);
        // Convert to rad/sec
        drivetrain.swerve(0, 0, Math.toRadians(rotate));
    }
}
