SwerveLib
=========

```mermaid
---
title: Swerve Lib Class Diagram
---
classDiagram
    DriverBase <|-- Driver
    RotatorBase <|-- Rotator
    Driver *-- SwerveModule
    Rotator *-- SwerveModule
    SwerveModule *-- SwerveDrivetrain : x4
    StopCommand --> SwerveDrivetrain
    RelativeSwerveCommand --> SwerveDrivetrain
    SwerveToPositionCommand --> SwerveDrivetrain
    RotateToHeadingCommand --> SwerveDrivetrain
    class DriverBase{
        -PIDController pid
        +setSpeed()
    }
    class Driver{
        -XYZMotor motor
        -XYZEncoder encoder
        +setVoltage()
    }
    class RotatorBase{
        -PIDController pid
        +setAngle()
    }
    class Rotator{
        -XYZMotor motor
        -XYZEncoder encoder
        +setVoltage()
    }
    class SwerveModule{
        +drive(angle, speed)
    }
    class SwerveDrivetrain{
        -SwerveDriveKinematics kinematics
        -SwerveDrivePoseEstimator odometry
        +swerve(vx, vy, vrot)
        +getPosition() Pose2D
        +followTrajectory(trajectory) SwerveCommand
    }
    class RelativeSwerveCommand{
        -XBoxController joystick
    }
    class SwerveToPositionCommand{
      -double x
      -double y
    }
    class RotateToHeadingCommand{
      -double heading
    }
```

