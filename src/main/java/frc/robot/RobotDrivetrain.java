// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveModule;

/** SwerveDrivetrain with Rotator, Driver and gyro */
public class RobotDrivetrain extends SwerveDrivetrain
{
    private final Pigeon2 gyro = new Pigeon2(0);

    public RobotDrivetrain()
    {
        super(0.53,
              0.58,
              new SwerveModule(new Rotator(0, RobotMap.FRONT_LEFT_ROTATE,  RobotMap.FRONT_LEFT_ANGLE, 120.7),
                               new Driver (0, RobotMap.FRONT_LEFT_DRIVE)),
              new SwerveModule(new Rotator(1, RobotMap.FRONT_RIGHT_ROTATE, RobotMap.FRONT_RIGHT_ANGLE, 39.8),
                               new Driver (1, RobotMap.FRONT_RIGHT_DRIVE)),
              new SwerveModule(new Rotator(2, RobotMap.BACK_RIGHT_ROTATE,  RobotMap.BACK_RIGHT_ANGLE, -99.8),
                               new Driver (2, RobotMap.BACK_RIGHT_DRIVE)),
              new SwerveModule(new Rotator(3, RobotMap.BACK_LEFT_ROTATE,   RobotMap.BACK_LEFT_ANGLE, -50.9),
                               new Driver (3, RobotMap.BACK_LEFT_DRIVE))
              );
    }

    public double getRawHeading()
    {
        return gyro.getYaw().getValueAsDouble();
    }

    public double getPitch()
    {
        return gyro.getPitch().getValueAsDouble();
    }

    public double getRoll()
    {
        return gyro.getRoll().getValueAsDouble();
    }
}
