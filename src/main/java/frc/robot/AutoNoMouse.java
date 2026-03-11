// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.tools.AutoTools.createTrajectory;
import static frc.tools.AutoTools.followPathWeaver;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
import frc.tools.SequenceWithStart;

/** Auto-no-mouse routines */
public class AutoNoMouse
{
    /** Create all our auto-no-mouse commands */
    public static List<Command> createAutoCommands(AprilTagFieldLayout tags, SwerveDrivetrain drivetrain, FuelHandler fuel_handler)
    {
        // List of all auto commands
        final List<Command> autos = new ArrayList<>();

        // Each auto is created within a { .. block .. } so we get local variables for 'path' and the like.
        // Each auto should start with a VariableWaitCommand to allow coordination with other teams
        {   // Drive back, open intake
            SequentialCommandGroup auto = new SequentialCommandGroup();
            auto.setName("Back 1.5m and Open");
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
            Trajectory path = createTrajectory(true, 0, 0, 180,
                                                   -1.5, 0, 180);
            auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
            auto.addCommands(fuel_handler.openIntake());
            autos.add(auto);
        }

        {   // Start with nose at red hub, drive back, shoot
            SequentialCommandGroup auto = new SequenceWithStart("Nose@red,shoot", 13.01, 4.02, 180);
            auto.addCommands(new VariableWaitCommand());

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 14.03, 4.07).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));

            autos.add(auto);
        }

        {   // Start with nose at blue hub, drive back, shoot
            SequentialCommandGroup auto = new SequenceWithStart("Nose@blue,shoot", 3.54, 4.00, 0);
            auto.addCommands(new VariableWaitCommand());

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.24, 4.02).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));

            autos.add(auto);
        }

        {   // Start with nose at blue hub, drive back, shoot, pick up from outpost, shoot
            SequentialCommandGroup auto = new SequenceWithStart("Nose@blue,outpost", 3.54, 4.00, 0);

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.5, 4.0).asProxy());
            // Shoot
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));
            // Move to outpost
            Trajectory path = createTrajectory(true,  2.5, 4.00,  -90,
                                                                       0.81, 0.64,  180);
            auto.addCommands(fuel_handler.openIntake()
                .alongWith(drivetrain.followTrajectory(path, 180).asProxy()));

            // Back off outpost
            path = createTrajectory(true, 0.81, 0.64,  45,
                                                           2.08, 2.01,  45);
            auto.addCommands(drivetrain.followTrajectory(path, 45).asProxy());
            // Shoot
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));

            autos.add(auto);
        }

        {   // Start with nose at blue bottom trench, sweep center, bump, shoot
            SequentialCommandGroup auto = new SequenceWithStart("Nose@blue buttom trench, center, bump", 3.57, 0.63, 0);

            // Through trench, sweep center
            Trajectory path = createTrajectory(true,  3.57, 0.63,  0,
                                                                       6.37, 0.69,  0,
                                                                       7.94, 1.75, 96,
                                                                       7.50, 3.60, 96);
            Supplier<Rotation2d> angle = () ->
            {   // In trench, head to zero, otherwise 100
                if (drivetrain.getPose().getX() < 6  &&
                    drivetrain.getPose().getY() < 1)
                    return Rotation2d.fromDegrees(0);
               return Rotation2d.fromDegrees(100);
            };
            auto.addCommands(fuel_handler.openIntake()
                .alongWith(drivetrain.followTrajectory(path, angle).asProxy()));

            // From center across bump
            path = createTrajectory(true, 7.50, 3.60, -135,
                                                           5.60, 2.41,  180,
                                                           2.66, 3.28,  131);
            auto.addCommands(drivetrain.followTrajectory(path, 25).asProxy());
            // Shoot
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));

            autos.add(auto);
        }

        {   // Start with nose at blue top trench, sweep center, bump, shoot, depot
            SequentialCommandGroup auto = new SequenceWithStart("Nose@blue top trench, center, bump, depot", 3.58, 7.40, 0);

            // Through trench, sweep center
            Trajectory path = createTrajectory(true,  3.58, 7.40,    0,
                                                                       5.95, 7.40,    0,
                                                                       7.96, 6.36,  -90,
                                                                       7.71, 4.27, -112);
            Supplier<Rotation2d> angle = () ->
            {   // In trench, head to zero, otherwise -100
                if (drivetrain.getPose().getX() < 6  &&
                    drivetrain.getPose().getY() > 7)
                    return Rotation2d.fromDegrees(0);
               return Rotation2d.fromDegrees(-100);
            };
            auto.addCommands(fuel_handler.openIntake()
                .alongWith(drivetrain.followTrajectory(path, angle).asProxy()));

            // From center across bump
            path = createTrajectory(true, 7.71, 4.27,  133,
                                                           5.40, 5.57, -180,
                                                           2.60, 5.36, -180);
            auto.addCommands(drivetrain.followTrajectory(path, -34).asProxy());
            // Shoot
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));

            // To depot
            auto.addCommands(new RotateToHeadingCommand(drivetrain, 143).asProxy());
            path = createTrajectory(true, 2.60, 5.36, 143,
                                                           0.82, 5.91, 178);
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());

            // Back off depot
            path = createTrajectory(true, 0.82, 5.91, -40,
                                                           2.38, 5.61, -39);
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());

            // Shoot
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));

            autos.add(auto);
        }

        {   // Start with nose at red hub, drive back, shoot, then move to trench
            SequentialCommandGroup auto = new SequenceWithStart("Nose@red,shoot,trench", 13.03, 4.0, 180);
            auto.addCommands(new VariableWaitCommand());

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 14.20, 4.0).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));
            // Move to trench
            Trajectory path = createTrajectory(true, 14.20, 4.00,  90,
                                                                      13.68, 6.90, 135,
                                                                      12.92, 7.43, 180);
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());

            autos.add(auto);
        }

        {   // Start with nose at blue hub, drive back, shoot, then move to trench
            SequentialCommandGroup auto = new SequenceWithStart("Nose@blue,shoot,trench", 3.5, 4.0, 0);
            auto.addCommands(new VariableWaitCommand());

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.25, 4.0).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5));
            // Move to trench
            Trajectory path = createTrajectory(true,  2.25, 4.00,  90,
                                                                       2.85, 6.90,  45,
                                                                       3.60, 7.43,   0);
            auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());

            autos.add(auto);
        }

        {   // Start with nose at blue hub, drive back, shoot, then move to trench, pickup more from center
            SequentialCommandGroup auto = new SequenceWithStart("Nose@blue,shoot,trench, center", 3.5, 4.0, 0);
            auto.addCommands(new VariableWaitCommand());

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.25, 4.0).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(new AutoAim(tags, drivetrain).withTimeout(3).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(3));
            // Move to trench and through
            Trajectory path = createTrajectory(true,  2.25, 4.00,  90,
                                                                       2.85, 6.90,  45,
                                                                       3.60, 7.43,   0,
                                                                       6.42, 7.30,   0);
            auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
            // Sweep the center
            path = createTrajectory(true,  6.42, 7.30,   0,
                                                            7.30, 6.14, -45,
                                                            7.30, 4.06, -90,
                                                            7.30, 2.30, -90);
            auto.addCommands(drivetrain.followTrajectory(path, -90).asProxy());
            // Back via bump
            path = createTrajectory(true,  7.30, 2.30, 180,
                                                            2.62, 2.30, 180);
            auto.addCommands(drivetrain.followTrajectory(path, 45).asProxy());

            autos.add(auto);
        }

        {   // Drive inverted L and back
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

        {   // Drive a 1.5 square using SwerveToPositionCommand & RotateToHeadingCommand
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

        {   // Blue Bottom: Shoot, Pickup, Shoot
            SequentialCommandGroup auto = new SequenceWithStart("BBSPS", 3.57, 1.63, 180);
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 3.57, 1.63, 180));
            // Move over to target
            Trajectory path = createTrajectory(true, 3.57, 1.63, 120,
                                                    2.77, 2.72,  90,
                                                    3.00, 4.00,  60);
            auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
            auto.addCommands(new AutoAim(tags, drivetrain).asProxy());
            auto.addCommands(new PrintCommand("Shoot!"));
            auto.addCommands(new WaitCommand(2));
            // Sweep through center of field to pick up fuel
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
            auto.addCommands(new AutoAim(tags, drivetrain).asProxy());
            auto.addCommands(new PrintCommand("Shoot!"));
            auto.addCommands(new WaitCommand(2));
            auto.addCommands(new PrintCommand("Done."));
            autos.add(auto);
        }

        {   // Red Top: Shoot, Pickup, Shoot
            SequentialCommandGroup auto = new SequenceWithStart("RTSPS", 13.00, 6.37, 0);
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 13.00, 6.37, 0));
            // Move over to target
            Trajectory path = createTrajectory(true, 13.00, 6.37, 0,
                                                13.48, 5.40, -90,
                                                13.52, 3.99, -96
                                                );
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());
            auto.addCommands(new AutoAim(tags, drivetrain).asProxy());
            auto.addCommands(new PrintCommand("Shoot!"));
            auto.addCommands(new WaitCommand(2));
            // Sweep through center of field to pick up fuel
            auto.addCommands(new PrintCommand("Open intake"));
            path = createTrajectory(true,
                                    13.52, 3.99, -96,
                                  13.21, 0.94, -147,
                                  11.87, 0.62, -179,
                                  9.51, 1.12, 153,
                                  8.86, 1.96, 90,
                                  8.96, 2.43, 55,
                                  12.00, 2.45, -1,
                                  13.88, 3.29, 53);
            auto.addCommands(drivetrain.followTrajectory(path, 90).asProxy());
            auto.addCommands(new PrintCommand("Close intake"));
            // Aim and shoot
            auto.addCommands(new AutoAim(tags, drivetrain).asProxy());
            auto.addCommands(new PrintCommand("Shoot!"));
            auto.addCommands(new WaitCommand(2));
            auto.addCommands(new PrintCommand("Done."));
            autos.add(auto);
        }

        {   // Red Bottom: Shoot, Pickup, Shoot
            SequentialCommandGroup auto = new SequenceWithStart("RBSPS", 13.00, 1.67, 180);
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 13.00, 1.67, 180));
            // Move over to target
            Trajectory path = createTrajectory(true, 13.00, 1.67, 68,
                                                  13.64, 3.65, 92);
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());
            auto.addCommands(new AutoAim(tags, drivetrain).asProxy());
            auto.addCommands(new PrintCommand("Shoot!"));
            auto.addCommands(new WaitCommand(2));
            // Pick up fuel from our side
            path = createTrajectory(true,
                                       13.64, 3.66, -44,
                                       16.05, 2.09, 0);
            auto.addCommands(
                new ParallelCommandGroup(
                    drivetrain.followTrajectory(path, 0).asProxy(),
                    new WaitCommand(1.0).andThen(new PrintCommand("Open Intake"))
                ));

            auto.addCommands(new WaitCommand(0.5));
            auto.addCommands(new PrintCommand("Close intake"));

            path = createTrajectory(true,
                                    16.05, 2.09, 142,
                                    13.79, 3.87, 142);
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());

            // Aim and shoot
            auto.addCommands(new AutoAim(tags, drivetrain).asProxy());
            auto.addCommands(new PrintCommand("Shoot!"));
            auto.addCommands(new WaitCommand(2));
            auto.addCommands(new PrintCommand("Done."));
            autos.add(auto);
        }

        {   // Pathweaver example
            SequentialCommandGroup auto = new SequenceWithStart("Tour", 4.04, 7.440,-180);
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 4.04, 7.44, -180));
            auto.addCommands(followPathWeaver(drivetrain, "Tour", 0).asProxy());
            auto.addCommands(new PrintCommand("Done."));
            autos.add(auto);
        }

        {   // Drive by
            SequentialCommandGroup auto = new SequenceWithStart("Drive By", 3.3, 1, 0);
            auto.addCommands(new VariableWaitCommand());

            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 3.30, 1, 0));

            // Use AutoAim as angle supplier
            AutoAim aim = new AutoAim(tags, drivetrain);
            Supplier<Rotation2d> angle = () -> aim.computeAngle(drivetrain.getPose());
            auto.addCommands(new InstantCommand(aim::initialize));

            // Trajectory for driving, always angling to hub
            Trajectory path = createTrajectory(true, 3.3, 1, 90,
                                                                      3.3, 7, 90);
            Command drive_out = drivetrain.followTrajectory(path, angle).asProxy();
            path = createTrajectory(true, 3.3, 7, -90,
                                                           3.3, 1, -90);
            Command drive_back = drivetrain.followTrajectory(path, angle).asProxy();
            Command drive_and_aim = drive_out.andThen(drive_back);

            // After 1 sec, enable shooter and keep it on
            Command shoot = new WaitCommand(1.0).andThen( fuel_handler.shoot().repeatedly() );

            // Do this while driving, end when done driving
            auto.addCommands(new ParallelDeadlineGroup(drive_and_aim, shoot));

            autos.add(auto);
        }

        return autos;
    }
}
