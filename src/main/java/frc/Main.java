// Copyright (c) FIRST team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.RobotBase;

/** Java 'Main'. Modify this file to select which robot to run */
public final class Main
{
  public static void main(String... args)
  {
      // System.out.println("Hello, robot!");
      // RobotBase.startRobot(frc.demo.BlinkDemoRobot::new);
      // RobotBase.startRobot(frc.led.LEDRingDemoRobot::new);
      // RobotBase.startRobot(frc.demo.MotorSparkMiniDemoRobot::new);
      // RobotBase.startRobot(frc.demo.TankDriveTestRobot::new);
      // RobotBase.startRobot(frc.swervebot.SwerveBot::new);

      // Robot drivetrain
      // RobotBase.startRobot(() -> new frc.swervelib.DriverDemoRobot(new frc.robot.Driver(0, frc.robot.RobotMap.FRONT_LEFT_DRIVE)));
      // RobotBase.startRobot(() -> new frc.swervelib.RotatorDemoRobot(new frc.robot.Rotator(0, frc.robot.RobotMap.FRONT_LEFT_ROTATE,
                                                                                            //  frc.robot.RobotMap.FRONT_LEFT_ANGLE,
                                                                                            //  0.0)));

      // RobotBase.startRobot(frc.demo.DigitalFilterDemoRobot::new);
      RobotBase.startRobot(frc.robot.SpinnerTestRobot::new);
      // RobotBase.startRobot(frc.robot.IntakeTestRobot::new);
      // RobotBase.startRobot(frc.robot.FuelHandlerTestRobot::new);
      // RobotBase.startRobot(frc.robot.HoodDemoRobot::new);

      // RobotBase.startRobot(frc.robot.Robot::new);
  }
}
