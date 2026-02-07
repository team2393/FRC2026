// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveModule;

/** SwerveDrivetrain with PracticeRotator, ...Driver and gyro */
public class PracticeDrivetrain extends SwerveDrivetrain
{
    private final Pigeon2 gyro = new Pigeon2(0);

    public PracticeDrivetrain()
    {
        super(0.53,
              0.53,
              new SwerveModule(new PracticeRotator(0, RobotMap.FRONT_LEFT_ROTATE,  RobotMap.FRONT_LEFT_ANGLE, -150.1),
                               new PracticeDriver (0, RobotMap.FRONT_LEFT_DRIVE)),
              new SwerveModule(new PracticeRotator(1, RobotMap.FRONT_RIGHT_ROTATE, RobotMap.FRONT_RIGHT_ANGLE, 19.8),
                               new PracticeDriver (1, RobotMap.FRONT_RIGHT_DRIVE)),
              new SwerveModule(new PracticeRotator(2, RobotMap.BACK_RIGHT_ROTATE,  RobotMap.BACK_RIGHT_ANGLE, 83.3),
                               new PracticeDriver (2, RobotMap.BACK_RIGHT_DRIVE)),
              new SwerveModule(new PracticeRotator(3, RobotMap.BACK_LEFT_ROTATE,   RobotMap.BACK_LEFT_ANGLE, -133.7),
                               new PracticeDriver (3, RobotMap.BACK_LEFT_DRIVE))
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
