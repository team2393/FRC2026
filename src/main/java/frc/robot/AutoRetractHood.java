// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.swervelib.SwerveDrivetrain;

/** Command that retracts hood when robot is close to trench */
public class AutoRetractHood extends Command
{
    private static final double blue_zone = 4.63, red_zone = 11.92;

    private final SwerveDrivetrain drivetrain;

    private final NetworkTableEntry nt_auto_retract_hood = SmartDashboard.getEntry("AutoRetractHood");
    private final NetworkTableEntry nt_hood_setpoint = SmartDashboard.getEntry("HoodSetpoint");

    public AutoRetractHood(SwerveDrivetrain drivetrain)
    {
        this.drivetrain = drivetrain;
        nt_auto_retract_hood.setDefaultBoolean(true);
    }

    @Override
    public void execute()
    {
        if (nt_auto_retract_hood.getBoolean(true))
        {
            Translation2d pos = drivetrain.getPose().getTranslation();
            boolean near_trench_y = pos.getY() < 1  ||  pos.getY() > 7;
            if (near_trench_y  && (MathUtil.isNear(blue_zone, pos.getX(), 1.0) ||
                                MathUtil.isNear(red_zone,  pos.getX(), 1.0)))
                nt_hood_setpoint.setDouble(1);
        }
    }
}
