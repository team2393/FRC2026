// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.tools.AutoTools;
import frc.tools.CommandRobotBase;
import frc.camera.CameraHelper;
import frc.swervelib.RelativeSwerveCommand;

/** FRC2026 robot */
public class Robot extends CommandRobotBase
{
    /** XXX Pick k2026RebuiltWelded or k2026RebuiltAndymark */
    private final AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /** Track if hub is active */
    private final HubTimer hub_timer = new HubTimer();

    /** Power distribution board to monitor current */
    private final PowerDistribution power_dist = new PowerDistribution();

    /** Drivetrain and related commands */
    private final RobotDrivetrain drivetrain = new RobotDrivetrain();
    private final Command joydrive = new RelativeSwerveCommand(drivetrain);
    private final Command aim = new AimToHub(tags, drivetrain);
    // private final Command aim = new RotateToTarget("Front", drivetrain);

    // private final FuelHandler fuel_handler = new FuelHandler();

    /** Handle cameras */
    private final CameraHelper camera = new CameraHelper(tags, "Front", "FrontCamera",
                                                         0.34, -0.1, 0.16,
                                                         0.0,
                                                         -10.0);

    /** Auto-no-mouse options */
    private final SendableChooser<Command> autos = new SendableChooser<>();

    public Robot()
    {
        // Configure speeds
        RobotOI.MAX_METERS_PER_SEC = 1.0;
        RobotOI.MAX_ROTATION_DEG_PER_SEC = 180.0;

        // Bind controller buttons
        RobotOI.joystick.x().whileTrue(aim);
        // RobotOI.joystick.a().onTrue(fuel_handler.take_in());
        // RobotOI.joystick.y().onTrue(fuel_handler.shoot());

        // By default, drive, and allow bound buttons to select other modes
        drivetrain.setDefaultCommand(joydrive);

        // Power dist. info
        power_dist.clearStickyFaults();
        SmartDashboard.putData("Power", power_dist);

        autos.setDefaultOption("Nothing", new PrintCommand("Do nothing"));
        for (Command auto : AutoNoMouse.createAutoCommands(tags, drivetrain))
             autos.addOption(auto.getName(), auto);
        SmartDashboard.putData(autos);
    }

    @Override
    public void disabledPeriodic()
    {
        AutoTools.indicateStart(drivetrain, autos.getSelected());
    }

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        camera.updatePosition(drivetrain);
    }

    @Override
    public void teleopInit()
    {
        // Start tracking the hub state
        CommandScheduler.getInstance().schedule(hub_timer);
    }

    @Override
    public void autonomousInit()
    {
        CommandScheduler.getInstance().schedule(autos.getSelected());
    }
}
