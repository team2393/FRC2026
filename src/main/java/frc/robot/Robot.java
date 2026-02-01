// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.tools.AutoTools;
import frc.tools.CommandRobotBase;
import frc.camera.CameraHelper;
import frc.swervelib.AbsoluteSwerveCommand;
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
    private final Command absdrive = new AbsoluteSwerveCommand(drivetrain);
    private final Command aim = new AimToHub(tags, drivetrain);
    // private final Command aim = new RotateToTarget("Front", drivetrain);

    // TODO Eventually, use FuelHandler. For now preliminary Intake
    // private final FuelHandler fuel_handler = new FuelHandler();
    private final Intake intake = new Intake();

    /** Handle cameras */
    private final List<CameraHelper> cameras = List.of(
        new CameraHelper(tags, "Front",
                         0.34, -0.1, 0.16,
                        0.0,
                        -10.0),
        new CameraHelper(tags, "Back",
                        -0.34, -0.07, 0.16,
                        180.0,
                        -10.0));

    /** Auto-no-mouse options */
    private final SendableChooser<Command> autos = new SendableChooser<>();

    public Robot()
    {
        // Configure speeds
        // Robot needs >1m/s to run over the bump
        RobotOI.MAX_METERS_PER_SEC = 2.0;
        RobotOI.MAX_ROTATION_DEG_PER_SEC = 180.0;
        AutoTools.config = new TrajectoryConfig(1.5, 1.0);
        // AutoTools.config = new TrajectoryConfig(2.5, 1.5);
        // XXX AutoTools.config.addConstraint(new SwerveDriveKinematicsConstraint(...));

        // Bind controller buttons
        RobotOI.joystick.x().whileTrue(aim.repeatedly());
        // RobotOI.joystick.a().onTrue(fuel_handler.toggleIntake());
        // RobotOI.joystick.y().onTrue(fuel_handler.shoot());
        // Helper for creating auto paths: Print X, Y, Heading on button press
        RobotOI.joystick.b().onTrue(new InstantCommand(() ->
        {
            var pose = drivetrain.getPose();
            System.out.format("%.2f, %.2f, %.0f,\n",
                         pose.getTranslation().getX(),
                         pose.getTranslation().getY(),
                         pose.getRotation().getDegrees());
        }
        ));

        RobotOI.joystick.a().whileTrue(new InstantCommand(() -> intake.open(true)));
        RobotOI.joystick.a().whileFalse(new InstantCommand(() -> intake.open(false)));
        RobotOI.joystick.leftBumper().onTrue(new InstantCommand(()->
        {
            drivetrain.setDefaultCommand(joydrive);
            CommandScheduler.getInstance().schedule(joydrive);
        }));
        RobotOI.joystick.rightBumper().onTrue(new InstantCommand(()->
        {
            drivetrain.setDefaultCommand(absdrive);
            CommandScheduler.getInstance().schedule(absdrive);
        }));
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
        for (var camera : cameras)
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
