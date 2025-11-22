// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.swervebot;

import static frc.tools.AutoTools.createTrajectory;

import java.util.ArrayList;
import java.util.List;

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
  public static List<Command> createAutoCommands(SwerveDrivetrain drivetrain)
  {
    // List of all autonomouse commands
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
      auto.addCommands(drivetrain.followTrajectory(path, 0));
      autos.add(auto);
    }

    { // Drive inverted L
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("Inverted L");
      auto.addCommands(new VariableWaitCommand());

      // SwerveToPositionCommand & RotateToHeadingCommand are always absolute,
      // so reset position to zero
      auto.addCommands(new ResetPositionCommand(drivetrain));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.0, 0.0));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 90));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.0, 2.5));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.0, 0.0));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 0.0));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 0));

      autos.add(auto);
    }

    { // Similar to Inv L, using trajectory
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("Inv L Traj");
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
      Trajectory path = createTrajectory(true, 0,   0,    0,
                                               1.8, 0.15, 45,
                                               2.0, 2.5, 90);
      auto.addCommands(drivetrain.followTrajectory(path, 0));
      // Driving 'backwards', note that headings are used in reverse
      path =            createTrajectory(false, 2.0, 2.5, 90,
                                               1.8, 0.15, 45,
                                               0,   0,    0);
      auto.addCommands(drivetrain.followTrajectory(path, 0));
      autos.add(auto);
    }

    { // Drive a 1.5 square using just SwerveToPositionCommand
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("1.5m Square");
      auto.addCommands(new VariableWaitCommand());

      // SwerveToPositionCommand & RotateToHeadingCommand are always absolute,
      // so reset position to zero
      auto.addCommands(new ResetPositionCommand(drivetrain));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.5, 0.0));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.5, 1.5));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 1.5));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 0.0));

      autos.add(auto);
    }

    { // Drive a 1.5 square using SwerveToPositionCommand & RotateToHeadingCommand
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("1.5m Square (rot)");
      auto.addCommands(new VariableWaitCommand());

      // SwerveToPositionCommand & RotateToHeadingCommand are always absolute,
      // so reset position to zero
      auto.addCommands(new ResetPositionCommand(drivetrain));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.5, 0.0));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 90));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1.5, 1.5));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 180));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 1.5));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, -90));

      auto.addCommands(new SwerveToPositionCommand(drivetrain, 0.0, 0.0));
      auto.addCommands(new RotateToHeadingCommand(drivetrain, 0));

      autos.add(auto);
    }

    {
      // Blue Bottom: Move out, Shoot, Pickup, Shoot
      SequentialCommandGroup auto = new SequenceWithStart("BBMSPS", 0.51, 2.38, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 0.51, 2.38, 180));
      // Move out (back), then over to front of target
      Trajectory path = createTrajectory(true, 0.51, 2.38,   0,
                                               1.70, 3.50,  90,
                                               1.44, 5.54, 120);
      auto.addCommands(drivetrain.followTrajectory(path, 180));
      auto.addCommands(new PrintCommand("Shoot!"));
      auto.addCommands(new WaitCommand(2));
      // Pickup another ring from right behind
      auto.addCommands(new PrintCommand("Open intake"));
      Trajectory path2 = createTrajectory(true, 1.44, 5.54, 0,
                                                2.60, 5.54, 0);
      auto.addCommands(drivetrain.followTrajectory(path2, 180));
      auto.addCommands(new PrintCommand("Close intake"));
      // Move forward to target and shoot
      Trajectory path3 = createTrajectory(true, 2.60, 5.54, 180,
                                                1.44, 5.54, 180);
      auto.addCommands(drivetrain.followTrajectory(path3, 180));
      auto.addCommands(new PrintCommand("Shoot!"));
      auto.addCommands(new WaitCommand(2));
      auto.addCommands(new PrintCommand("Done."));
      autos.add(auto);
    }

    {
      // Follow a Pathweaver path
      SequentialCommandGroup auto = new SequenceWithStart("Circle", 0.15, 2.10, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 0.15, 2.10, 0));
      auto.addCommands(AutoTools.followPathWeaver(drivetrain, "Circle", 180));
      autos.add(auto);
    }

    return autos;
  }
}
