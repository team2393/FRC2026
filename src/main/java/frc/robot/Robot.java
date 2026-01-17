// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.tools.CommandRobotBase;
import frc.camera.RotateToTarget;
import frc.swervelib.RelativeSwerveCommand;

/** FRC2026 robot */
public class Robot extends CommandRobotBase
{
    private final AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private final RobotDrivetrain drivetrain = new RobotDrivetrain();
    private final Command joydrive = new RelativeSwerveCommand(drivetrain);
    private final Command aim = new AimToHub(tags, drivetrain);
    // private final Command aim = new RotateToTarget("Front", drivetrain);

    // private final FuelHandler fuel_handler = new FuelHandler();

    private final HubTimer hub_timer = new HubTimer();

    public Robot()
    {
        // Configure speeds
        RobotOI.MAX_METERS_PER_SEC = 1.0;
        RobotOI.MAX_ROTATION_DEG_PER_SEC = 180.0;

        RobotOI.joystick.x().whileTrue(aim);

        // RobotOI.joystick.a().onTrue(fuel_handler.take_in());
        // RobotOI.joystick.y().onTrue(fuel_handler.shoot());
    }

    @Override
    public void teleopInit()
    {
        CommandScheduler.getInstance().schedule(hub_timer);
        drivetrain.setDefaultCommand(joydrive);
    }
}
