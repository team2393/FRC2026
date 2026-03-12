// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.tools.ApplyAdjustableSettingCommand;
import frc.tools.ApplySettingsCommand;
import frc.tools.AutoTools;
import frc.tools.CommandRobotBase;
import frc.camera.CameraHelper;
import frc.swervelib.AbsoluteSwerveCommand;
import frc.swervelib.RelativeSwerveCommand;
import frc.swervelib.ResetHeadingCommand;
import frc.swervelib.SwerveDrivetrain;

/** FRC2026 robot */
public class Robot extends CommandRobotBase
{
    /** TODO Pick k2026RebuiltWelded or k2026RebuiltAndymark */
    private final AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /** Track if hub is active */
    private final HubTimer hub_timer = new HubTimer();

    /** Power distribution board to monitor current */
    // private final PowerDistribution power_dist = new PowerDistribution();

    /** Drivetrain and related commands */
    private final SwerveDrivetrain drivetrain = RobotMap.is_practice_chassis
                                              ? new PracticeDrivetrain()
                                              : new RobotDrivetrain();
    // private final SwerveDrivetrain drivetrain = new PracticeDrivetrain();
    private final Command reset_heading = new ResetHeadingCommand(drivetrain);
    private final Command joydrive = new RelativeSwerveCommand(drivetrain);
    private final Command absdrive = new AbsoluteSwerveCommand(drivetrain);
    private final Command aim = new AutoAim(tags, drivetrain);
    private final Command pass = new ApplyAdjustableSettingCommand("", "PassHood", 30, "HoodSetpoint")
                        .andThen(new ApplyAdjustableSettingCommand("", "PassSpinner", 2000, "SpinnerSetpoint"))
                        .withName("Pass");
    private final Command auto_retract_hood = new AutoRetractHood(drivetrain);

    private final FuelHandler fuel_handler = new FuelHandler();
    private final Hood hood = new Hood();

    /** Handle cameras */
    private final List<CameraHelper> cameras = List.of(
        new CameraHelper(tags, "Front",
                        -0.11, -0.22, 0.46,
                        0.0,
                        -20.0),
        new CameraHelper(tags, "Back",
                         -0.32, -0.01, 0.27,
                        180.0,
                        -10.0)
                        );

    /** Auto-no-mouse options */
    private final SendableChooser<Command> autos = new SendableChooser<>();

    public Robot()
    {
        System.out.println("** RIO serial " + RobotController.getSerialNumber());
        System.out.println("** on " + drivetrain.getClass().getName());
        System.out.println("************************************");

        // Configure speeds
        // Robot needs >1m/s to run over the bump
        // Max speed used in teleop
        RobotOI.MAX_METERS_PER_SEC = 2.0;
        RobotOI.MAX_ROTATION_DEG_PER_SEC = 180.0;
        // Max speed used in auto
        AutoTools.config = new TrajectoryConfig(1.0, 3.0);
        // AutoTools.config = new TrajectoryConfig(2.5, 3.0);
        if (RobotBase.isSimulation())
            AutoTools.config = new TrajectoryConfig(2.0, 3.0);
        AutoTools.config.addConstraint(new SwerveDriveKinematicsConstraint(drivetrain.getKinematics(),
                                                                           SwerveDrivetrain.MAX_METERS_PER_SEC));

        hood.reset();

        // Bind controller buttons
        RobotOI.joystick.x().whileTrue(aim.repeatedly());
        RobotOI.joystick.a().onTrue(fuel_handler.toggleIntake());
        RobotOI.joystick.y().whileTrue(fuel_handler.keepShooting());

        ApplySettingsCommand trench = new ApplySettingsCommand("Trench");
        trench.add("HoodSetpoint", 1);
        SmartDashboard.putData(trench);

        SmartDashboard.putData("Pass", pass);
        // Helper for creating auto paths: Print X, Y, Heading on button press
        AtomicInteger pos_index = new AtomicInteger();
        RobotOI.joystick.b().onTrue(new InstantCommand(() ->
        {
            var pose = drivetrain.getPose();
            System.out.format("%2d) %.2f, %.2f, %.0f,\n",
                         pos_index.incrementAndGet(),
                         pose.getTranslation().getX(),
                         pose.getTranslation().getY(),
                         pose.getRotation().getDegrees());
        }
        ));

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
        SmartDashboard.putData("Reset", reset_heading);
        // By default, drive, and allow bound buttons to select other modes
        drivetrain.setDefaultCommand(joydrive);

        // Power dist. info
        // power_dist.clearStickyFaults();
        // SmartDashboard.putData("Power", power_dist);

        // Auto options
        autos.setDefaultOption("Nothing", new PrintCommand("Do nothing"));
        for (Command auto : AutoNoMouse.createAutoCommands(tags, drivetrain, fuel_handler))
            autos.addOption(auto.getName(), auto);
        SmartDashboard.putData(autos);
    }

    @Override
    public void disabledPeriodic()
    {
        hub_timer.cancel();
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
        CommandScheduler.getInstance().schedule(auto_retract_hood);
    }

    @Override
    public void autonomousInit()
    {
        CommandScheduler.getInstance().schedule(autos.getSelected());
    }
}
