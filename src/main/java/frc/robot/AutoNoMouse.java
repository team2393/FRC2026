// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static frc.tools.AutoTools.createTrajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    // Field size:
    // "Welded"   length 16.541, width 8.069
    // "Andymark" length 16.518, width 8.043
    //  Average          16.530        8.056
    //
    // To mirror red <-> blue coordinates:
    //
    // x     ->   16.53 - x
    // y     ->   8.056 - y
    // angle ->     180 + angle

    private static Command shootAndWiggle(FuelHandler fuel_handler)
    {
        return new ParallelCommandGroup(fuel_handler.keepShooting(),
                                        fuel_handler.agitateIntake());
    }

    /** Create all our auto-no-mouse commands */
    public static List<Command> createAutoCommands(SwerveDrivetrain drivetrain, FuelHandler fuel_handler,AutoAim auto_aim)
    {
        // List of all auto commands
        final List<Command> autos = new ArrayList<>();

        // Each auto is created within a { .. block .. } so we get local variables for 'path' and the like.
        // Each auto should start with a VariableWaitCommand to allow coordination with other teams
        {   // Drive back, open intake
            SequentialCommandGroup auto = new SequentialCommandGroup();
            auto.setName("Back 1.5m and Open *universal*");
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
            Trajectory path = createTrajectory(true, 0, 0, 180,
                                                   -1.5, 0, 180);
            auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
            auto.addCommands(fuel_handler.openIntake());
            autos.add(auto);
        }

        {   // Drive back, shoot
            SequentialCommandGroup auto = new SequentialCommandGroup();
            auto.setName("Back and shoot *universal*");
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
            Trajectory path = createTrajectory(true, 0, 0, 180,
                                                   -1.5, 0, 180);
            auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());
            auto.addCommands(auto_aim.aimOnce().asProxy().andThen(fuel_handler.keepShooting()));
            autos.add(auto);
        }

        {   // Start with nose at blue hub, drive back, shoot
            SequentialCommandGroup auto = new SequenceWithStart("@blue hub,shoot", 3.54, 4.00, 0);
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.24, 4.02).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().repeatedly());

            autos.add(auto);
        }

        {   // Start with nose at red hub, drive back, shoot
            SequentialCommandGroup auto = new SequenceWithStart("@red hub,shoot", 13.01, 4.02, 180);
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 14.03, 4.07).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().repeatedly());

            autos.add(auto);
        }

        {   // Test Start with nose at blue hub, drive back, shoot, pick up from outpost, shoot
            SequentialCommandGroup auto = new SequenceWithStart("@blue hub, outpost", 3.54, 4.00, 0);
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.5, 4.0).asProxy());
            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5).andThen(fuel_handler.store()));
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
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().repeatedly());

            autos.add(auto);
        }

        {   // Start with nose at red hub, drive back, shoot, pick up from outpost, shoot
            SequentialCommandGroup auto = new SequenceWithStart("@red hub, outpost", 16.53-3.54, 8.056-4.00, 180);
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 16.53-2.5, 8.056-4.0).asProxy());
            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5).andThen(fuel_handler.store()));
            // Move to outpost
            Trajectory path = createTrajectory(true,  16.53-2.5, 8.056-4.00,  180-90,
                                                              16.53-0.81, 8.056-0.64,  0);
            auto.addCommands(fuel_handler.openIntake()
                .alongWith(drivetrain.followTrajectory(path, 0).asProxy()));

            // Back off outpost
            path = createTrajectory(true, 16.53-0.81, 8.056-0.64,  180+45,
                                                  16.53-2.08, 8.056-2.01,  180+45);
            auto.addCommands(drivetrain.followTrajectory(path, 180+45).asProxy());
            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().repeatedly());

            autos.add(auto);
        }

        {   // Start with nose at blue bottom trench, sweep center, bump, shoot
            SequentialCommandGroup auto = new SequenceWithStart("@blue outpostside trench, center, bump", 3.57, 0.63, 0);
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

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
                                                           1.95, 3.35,  131);
            auto.addCommands(drivetrain.followTrajectory(path, 15).asProxy());
            //wait 1s
            auto.addCommands(new WaitCommand(1.0));
            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.keepShooting());

            autos.add(auto);
        }

        {
            SequentialCommandGroup auto = new SequenceWithStart("NEW @red outpostside trench, center, bump", 16.53-3.57, 8-0.63-0.1, 180);
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Through trench, sweep center
            Trajectory path = createTrajectory(true,  16.53-3.57, 8-0.63-0.1,  180,
                                                                       16.53-6.37, 8-0.69, 180,
                                                                       16.53-7.94, 8-1.75, 180+96,
                                                                       16.53-7.50, 8-3.60, 180+96);
            Supplier<Rotation2d> angle = () ->
            {   // In trench, head to zero, otherwise 100
                if (drivetrain.getPose().getX() >16.53-6  &&
                    drivetrain.getPose().getY() > 7)
                    return Rotation2d.fromDegrees(180);
               return Rotation2d.fromDegrees(180+100);
            };
            auto.addCommands(fuel_handler.openIntake()
                .alongWith(drivetrain.followTrajectory(path, angle).asProxy()));

            // From center across bump
            path = createTrajectory(true, 16.53-7.50, 8-3.60, 180-135,
                                                           16.53-5.60, 8-2.41,  0,
                                                           14.65, 4.98,  180+131);
            auto.addCommands(drivetrain.followTrajectory(path, 180+25).asProxy());
            //wait 1s
            auto.addCommands(new WaitCommand(1.0));
            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.keepShooting());

            autos.add(auto);
        }


        {   // Start with nose at red bottom trench, sweep center, bump, shoot
            SequentialCommandGroup auto = new SequenceWithStart("@red depotside trench, center, bump", 16.53-3.57, 0.63, 180);
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Through trench, sweep center
            Trajectory path = createTrajectory(true,  16.53-3.57, 0.63, 180,
                                                              16.53-6.37, 0.69, 180,
                                                              16.53-7.94, 1.75, 180-96,
                                                              16.53-7.50, 3.60, 180-96);
            Supplier<Rotation2d> angle = () ->
            {   // In trench, head to zero, otherwise 100
                if (drivetrain.getPose().getX() > 16.53-6  &&
                    drivetrain.getPose().getY() < 1)
                    return Rotation2d.fromDegrees(180);
               return Rotation2d.fromDegrees(180-100);
            };
            auto.addCommands(fuel_handler.openIntake()
                .alongWith(drivetrain.followTrajectory(path, angle).asProxy()));

            // From center across bump
            path = createTrajectory(true, 16.53-7.50, 3.60, 180+135,
                                                  16.53-5.60, 2.41,  0,
                                                   14.83, 2.93, 10);
            auto.addCommands(drivetrain.followTrajectory(path, 159).asProxy());
            //wait 1s
            auto.addCommands(new WaitCommand(1.0));
            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().repeatedly());

            autos.add(auto);
        }

        {   // Start with nose at blue top trench, sweep center, bump, shoot, depot
            SequentialCommandGroup auto = new SequenceWithStart("@blue depotside trench, center, bump, depot", 3.58, 7.30, 0);
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Through trench, sweep center
            Trajectory path = createTrajectory(true,  3.58, 7.30,    0,
                                                                       5.95, 7.30,    0,
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
                                                           5.40, 5.50, -180,
                                                           2.00, 5.36, -180);
            auto.addCommands(drivetrain.followTrajectory(path, -34).asProxy());
            //wait 1s
            auto.addCommands(new WaitCommand(1.0));
            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5).andThen(fuel_handler.store()));

            // To depot
            auto.addCommands(new RotateToHeadingCommand(drivetrain, 143).asProxy());
            path = createTrajectory(true, 2.00, 5.36, 143,
                                                           0.82, 5.91, 178);
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());

            // Back off depot
            path = createTrajectory(true, 0.82, 5.91, -40,
                                                           2.38, 5.61, -39);
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());

            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.keepShooting());
            // TODO try this: auto.addCommands(shootAndWiggle(fuel_handler));

            autos.add(auto);
        }

        {   // Start with nose at blue bottom bump, sweep center, trench, outpost
            SequentialCommandGroup auto = new SequenceWithStart("@blue outpostside bump, center, outpost", 3.55, 2.36, 0);
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Back off from bump and get to 45 degree to then cross the bump
            Trajectory path = createTrajectory(true,  3.55, 2.36,    180,
                                                                       2.50, 2.36,    180);
            auto.addCommands(fuel_handler.openIntake()
                .alongWith(drivetrain.followTrajectory(path, -45).asProxy()));

            // Cross bump, sweep center, move to outpost
            path = createTrajectory(true,  2.50, 2.36,    0,
                                                            6.21, 2.39,    0,
                                                            7.24, 4.07,   -2,
                                                            7.69, 2.08, -100,
                                                            5.90, 0.62,  180,
                                                            3.19, 0.64,  180,
                                                            0.68, 0.66,  180);
            Supplier<Rotation2d> angle = () ->
            {   // In trench, head to base
                if (drivetrain.getPose().getX() < 8.1  &&
                    drivetrain.getPose().getY() < 2.0)
                    return Rotation2d.fromDegrees(180);
                // Still on bump?
                if (drivetrain.getPose().getX() < 5)
                    return Rotation2d.fromDegrees(-45);
                // Sweep center
               return Rotation2d.fromDegrees(-100);
            };
            auto.addCommands(drivetrain.followTrajectory(path, angle).asProxy());

            // Back off outpost
            path = createTrajectory(true, 0.67, 0.66, 36,
                                                           2.63, 2.38, 40);
            auto.addCommands(drivetrain.followTrajectory(path, 40).asProxy());

            // Shoot
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().repeatedly());

            autos.add(auto);
        }

        {   // Start with nose at red hub, drive back, shoot, then move to trench
            SequentialCommandGroup auto = new SequenceWithStart("@red outpost,shoot,trench", 13.03, 4.0, 180);
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 14.20, 4.0).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5).andThen(fuel_handler.store()));
            // Move to trench
            Trajectory path = createTrajectory(true, 14.20, 4.00,  90,
                                                                      13.68, 6.90, 135,
                                                                      12.92, 7.43, 180);
            auto.addCommands(drivetrain.followTrajectory(path, 180).asProxy());

            autos.add(auto);
        }

        {   // Start with nose at blue hub, drive back, shoot, then move to trench
            SequentialCommandGroup auto = new SequenceWithStart("@blue outpost,shoot,trench", 3.5, 4.0, 0);
            auto.addCommands(new VariableWaitCommand());
            auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

            // Drive back
            auto.addCommands(new SwerveToPositionCommand(drivetrain, 2.25, 4.0).asProxy());
            // Shoot
            auto.addCommands(fuel_handler.openIntake());
            auto.addCommands(auto_aim.aimOnce().withTimeout(5).asProxy());
            auto.addCommands(fuel_handler.shoot().withTimeout(5).andThen(fuel_handler.store()));
            // Move to trench
            Trajectory path = createTrajectory(true,  2.25, 4.00,  90,
                                                                       2.85, 6.90,  45,
                                                                       3.60, 7.43,   0);
            auto.addCommands(drivetrain.followTrajectory(path, 0).asProxy());

            autos.add(auto);
        }

        return autos;
    }
}
