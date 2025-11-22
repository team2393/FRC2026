// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.swervebot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.demo.DemoMechanism;
import frc.demo.DemoMechanismArmCommand;
import frc.demo.DemoMechanismLiftCommand;
import frc.demo.DemoMechanismPokeCommand;
import frc.led.ColorPair;
import frc.led.Comet;
import frc.led.Fill;
import frc.led.Rainbow;
import frc.led.LEDRing;
import frc.led.Marquee;
import frc.swervelib.AbsoluteSwerveCommand;
import frc.swervelib.RelativeSwerveCommand;
import frc.swervelib.ResetHeadingCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveOI;
import frc.swervelib.SwerveToPositionCommand;
import frc.tools.AutoTools;
import frc.tools.CommandRobotBase;

/** Swerve demo robot */
public class SwerveBot extends CommandRobotBase
{
  private final SwervebotDrivetrain drivetrain = new SwervebotDrivetrain();
  private final Command relswerve = new RelativeSwerveCommand(drivetrain);
  private final Command absswerve = new AbsoluteSwerveCommand(drivetrain);
  // private final Command center_on_tag = new CenterOnAprilTag(drivetrain);
  
  private final LEDRing ring = new LEDRing();

  private final DemoMechanism mechanism = new DemoMechanism();
  private final Command mechanism_demo =
    new RepeatCommand
    (
      new SequentialCommandGroup
      (
        new DemoMechanismArmCommand(mechanism, -80, 180, 2),
        new DemoMechanismPokeCommand(mechanism, true),
        new WaitCommand(1.0),
        new ParallelCommandGroup
        (
          new DemoMechanismArmCommand(mechanism, 180, 0, 1),
          new DemoMechanismLiftCommand(mechanism, 0.25, 0.5, 3)
        ),
        new DemoMechanismPokeCommand(mechanism, false),
        new WaitCommand(1.0),
        new ParallelCommandGroup
        (
          new DemoMechanismArmCommand(mechanism, 0, -80, 1),
          new DemoMechanismLiftCommand(mechanism, 0.5, 0.25, 1)

        ),
        new WaitCommand(5)
      )
    );
  
  private final SendableChooser<Command> autos = new SendableChooser<>();

  public SwerveBot()
  {
    // Slower, smoother movement
    SwerveOI.MAX_METERS_PER_SEC = SwerveDrivetrain.MAX_METERS_PER_SEC = 1.0;
    SwerveOI.MAX_ROTATION_DEG_PER_SEC = SwerveDrivetrain.MAX_ROTATION_DEG_PER_SEC = 45;
    SwerveOI.forward_slew = new SlewRateLimiter(1.5);
    SwerveOI.side_slew = new SlewRateLimiter(1.5);
    SwerveOI.rotation_slew = new SlewRateLimiter(90);

    SwerveToPositionCommand.MAX_SPEED = SwerveToPositionCommand.ACCEL = 1.0;

    SwerveOI.reset();

    ring.setDefaultCommand(
      new RepeatCommand(
        new SequentialCommandGroup(
          new Marquee(ring).withTimeout(5),
          new Fill(ring).withTimeout(5),
          new Comet(ring).withTimeout(5),
          new ColorPair(ring, Color.kDarkGreen, Color.kDarkGoldenrod).withTimeout(5),
          new Rainbow(ring).withTimeout(5)
        )));
          
    autos.setDefaultOption("Nothing", new PrintCommand("Do nothing"));

    for (Command auto : AutoNoMouse.createAutoCommands(drivetrain))
      autos.addOption(auto.getName(), auto);
    SmartDashboard.putData(autos);
  }
  
  @Override
  public void disabledPeriodic()
  {
    AutoTools.indicateStart(drivetrain, autos.getSelected());
  }  

  @Override
  public void teleopInit()
  {
    // Bind buttons to commands
    drivetrain.setDefaultCommand(relswerve);
    SwerveOI.selectRelative().onTrue(relswerve);
    SwerveOI.selectAbsolute().onTrue(absswerve);
    SwerveOI.resetDrivetrain().onTrue(new ResetHeadingCommand(drivetrain));
    // SwerveOI.joystick.a().whileTrue(center_on_tag);

    mechanism_demo.schedule();
  }

  @Override
  public void autonomousInit()
  {
    autos.getSelected().schedule();
  }
}
