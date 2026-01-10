// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervebot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command for rotating to target */
public class RotateToTarget extends Command
{
    private final Camera2D camera;
    private final SwervebotDrivetrain drivetrain;
    private final PIDController pid = new PIDController(2.0, 1.0, 0);
        
    public RotateToTarget(Camera2D camera, SwervebotDrivetrain drivetrain)
    {
        this.camera = camera;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        SmartDashboard.putData("rotate_to_target", pid);
    }
    
    @Override
    public void execute()
    {
        double target_angle = camera.getAngleToTarget();
        if (Double.isNaN(target_angle))
        {
            drivetrain.stop();
            System.out.println("No camera reading!");
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
