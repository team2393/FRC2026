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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    private final Command pass = new ApplyAdjustableSettingCommand("", "PassHood", 30, "HoodSetpoint")
                        .andThen(new ApplyAdjustableSettingCommand("", "PassSpinner", 2000, "SpinnerSetpoint"))
                        .withName("Pass");


    private final FuelHandler fuel_handler = new FuelHandler();
    private final Hood hood = new Hood();
    private final Command auto_retract_hood = new AutoRetractHood(drivetrain);
    private final AutoAim auto_aim = new AutoAim(tags, drivetrain);

    /** Handle cameras */
    private final List<CameraHelper> cameras = List.of(
        new CameraHelper(tags, "Front",
                        -0.14, -0.22, 0.43,
                        0.0,
                        -20.0),
        new CameraHelper(tags, "Back",
                         -0.33, 0.00, 0.23,
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
        AutoTools.config = new TrajectoryConfig(1.5, 3.0);
        // AutoTools.config = new TrajectoryConfig(2.5, 3.0);
        if (RobotBase.isSimulation())
            AutoTools.config = new TrajectoryConfig(2.0, 3.0);
        AutoTools.config.addConstraint(new SwerveDriveKinematicsConstraint(drivetrain.getKinematics(),
                                                                           SwerveDrivetrain.MAX_METERS_PER_SEC));

        hood.reset();

        // Bind controller buttons
        RobotOI.joystick.x().whileTrue(auto_aim.aimContinuously());
        RobotOI.joystick.a().onTrue(fuel_handler.toggleIntake());
        RobotOI.joystick.y().whileTrue(fuel_handler.keepShooting());

        // Aim, then shoot (falling back to default drive command)
        Command aim_then_shoot = auto_aim.aimOnce().asProxy().andThen(fuel_handler.keepShooting());

        // Aim, then continue to aim (..and drive) while shooting,
        Command aim_while_shooting =
            auto_aim.aimOnce().withTimeout(1.0)
                              .andThen(new ParallelCommandGroup(auto_aim.aimContinuously(),
                                                                fuel_handler.keepShooting()));

        // Pick one or the other based on "AimWhileShooting" on dashboard
        NetworkTableEntry do_aim_while_shooting = SmartDashboard.getEntry("AimWhileShooting");
        do_aim_while_shooting.setDefaultBoolean(true);
        RobotOI.joystick.rightTrigger().whileTrue(
            new ConditionalCommand(aim_while_shooting,
                                   aim_then_shoot,
                                   () -> do_aim_while_shooting.getBoolean(true)));

        RobotOI.buttonboard.button(4).onTrue(fuel_handler.openIntake());
        RobotOI.buttonboard.button(9).onTrue(fuel_handler.closeIntake());

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
        for (Command auto : AutoNoMouse.createAutoCommands(drivetrain, fuel_handler, auto_aim))
            autos.addOption(auto.getName(), auto);
        SmartDashboard.putData(autos);
    }

    @Override
    public void disabledInit()
    {
        hub_timer.cancel();
        // Directly run command
        fuel_handler.stopShooting().initialize();
    }

    /** While disabled, we like to show the start position of the currently selected auto.
     *  During a match, the sequence is this:
     *  1) Disabled - Show the auto start pos!
     *  2) Auto
     *  3) Briefly disabled - Leave position alone!!
     *  4) Teleop
     *
     *  In step 1, and also after later teleop test runs,
     *  we want to show the auto start position.
     *  In step 3, however, we MUST NOT change the actual
     *  robot position into the auto start pos.
     *  This flag tells us if we're coming out of auto (2) into step 3.
     */
    private boolean was_in_teleop = true;

    @Override
    public void disabledPeriodic()
    {
        if (was_in_teleop)
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
        was_in_teleop = true;
        // Start tracking the hub state
        CommandScheduler.getInstance().schedule(hub_timer);
        CommandScheduler.getInstance().schedule(auto_retract_hood);
    }

    @Override
    public void autonomousInit()
    {
        was_in_teleop = false;
        CommandScheduler.getInstance().schedule(auto_retract_hood);
        CommandScheduler.getInstance().schedule(autos.getSelected());
    }
}
