// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.tools.AutoTools.createTrajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.swervelib.ResetPositionCommand;
import frc.swervelib.SelectAbsoluteTrajectoryCommand;
import frc.swervelib.SelectRelativeTrajectoryCommand;
import frc.swervelib.RotateToHeadingCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveToPositionCommand;
import frc.swervelib.VariableWaitCommand;
import frc.tools.AutoTools;
import frc.tools.SequenceWithStart;

/** Auto-no-mouse routines */
public class AutoNoMouse
{
  /** Create all our auto-no-mouse commands */
  public static List<Command> createAutoCommands(AprilTagFieldLayout tags, SwerveDrivetrain drivetrain)
  {
    // List of all auto commands
    final List<Command> autos = new ArrayList<>();

    // Each auto is created within a { .. block .. } so we get local variables for 'path' and the like.
    // Each auto should start with a VariableWaitCommand to allow coordination with other teams
    { // Drive forward 2.0 m using a (simple) trajectory
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("Forward 2.0m");
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
      Trajectory path = createTrajectory(true, 0,   0, 0,
                                               2.0, 0, 0);
      auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
      autos.add(auto);
    }

    { // Drive inverted L and back
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("Inverted L");
      auto.addCommands(new VariableWaitCommand());

      // SwerveToPositionCommand & RotateToHeadingCommand are always absolute,
      // so reset position to zero
      auto.addCommands(new ResetPositionCommand(drivetrain));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.0, 0.0).asProxy());
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 90).asProxy());
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.0, 2.5).asProxy());
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.0, 0.0).asProxy());
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 0.0).asProxy());
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 0).asProxy());

      autos.add(auto);
    }

    { // Drive a 1.5 square using SwerveToPositionCommand & RotateToHeadingCommand
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("1.5m Square");
      auto.addCommands(new VariableWaitCommand());

      // SwerveToPositionCommand & RotateToHeadingCommand are always absolute,
      // so reset position to zero
      auto.addCommands(new ResetPositionCommand(drivetrain));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.5, 0.0).asProxy());
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 90).asProxy());

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.5, 1.5).asProxy());
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 180).asProxy());

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 1.5).asProxy());
      auto.addCommands(new RotateToHeadingCommand(drivetrain, -90).asProxy());

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 0.0).asProxy());
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 0).asProxy());

      autos.add(auto);
    }

    {
      // Blue Bottom: Shoot, Pickup, Shoot
      SequentialCommandGroup auto = new SequenceWithStart("BBSPS", 3.57, 1.63, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 3.57, 1.63, 180));
      // Move out (back), then over to front of target
      Trajectory path = createTrajectory(true, 3.57, 1.63, 120,
                                               2.77, 2.72,  90,
                                               3.00, 4.00,  60);
      auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
      auto.addCommands(new AimToHub(tags, drivetrain).asProxy());
      auto.addCommands(new PrintCommand("Shoot!"));
      auto.addCommands(new WaitCommand(2));
      // Pickup another ring from right behind
      auto.addCommands(new PrintCommand("Open intake"));
      path = createTrajectory(true, 3.00, 4.00, -90,
                                                2.50, 2.30, -90,
                                                2.32, 1.34, -45,
                                                4.60, 0.67, 0,
                                                7.10, 1.30, 45,
                                                8.00, 2.70, 100,
                                                7.40, 3.36, 180,
                                                6.80, 3.00, -90,
                                                4.60, 2.36, -180,
                                                2.66, 3.04, 147);
      auto.addCommands(drivetrain.followTrajectory(path, 90).asProxy());
      auto.addCommands(new PrintCommand("Close intake"));
      // Aim and shoot
      auto.addCommands(new AimToHub(tags, drivetrain).asProxy());
      auto.addCommands(new PrintCommand("Shoot!"));
      auto.addCommands(new WaitCommand(2));
      auto.addCommands(new PrintCommand("Done."));
      autos.add(auto);
    }

    return autos;
  }
}
