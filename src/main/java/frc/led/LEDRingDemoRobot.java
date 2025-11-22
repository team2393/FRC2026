// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.tools.CommandRobotBase;

public class LEDRingDemoRobot extends CommandRobotBase
{
  private final LEDRing ring = new LEDRing();

  public LEDRingDemoRobot()
  {
    ring.setDefaultCommand(new Comet(ring));
    // ring.setDefaultCommand(new ColorPair(ring, ));
  }

  @Override
  public void teleopInit()
  {
    // Roll red and green around the ring for 3 seconds
    Command roll = new ParallelRaceGroup(
                      new ColorPair(ring, Color.kRed, Color.kGreen),
                      new WaitCommand(3)
                    );

    // Alternate so-called "fluent" way of getting the same end result, different colors:
    Command roll2 = new ColorPair(ring, Color.kDarkGreen, Color.kDarkGoldenrod).withTimeout(3);

    // Blink red/green a few times
    Command blink = new SequentialCommandGroup(
                          new SetToRGB(ring, 255, 0, 0),
                          new WaitCommand(0.5),
                          new SetToRGB(ring, 0, 255, 0),
                          new WaitCommand(0.5),
                          new SetToRGB(ring, 255, 0, 0),
                          new WaitCommand(0.5),
                          new SetToRGB(ring, 0, 255, 0),
                          new WaitCommand(0.5),
                          new SetToRGB(ring, 255, 0, 0),
                          new WaitCommand(0.5),
                          new SetToRGB(ring, 0, 255, 0),
                          new WaitCommand(0.5)
                        );

    // "fluent" alternative
    // blink =    new SetToRGB(ring, 255, 0, 0)
    //   .andThen(Commands.waitSeconds(0.5))
    //   .andThen(new SetToRGB(ring, 0, 255, 0))
    //   .andThen(Commands.waitSeconds(0.5))
    //   .andThen(new SetToRGB(ring, 255, 0, 0))
    //   .andThen(Commands.waitSeconds(0.5))
    //   .andThen(new SetToRGB(ring, 0, 255, 0))
    //   .andThen(Commands.waitSeconds(0.5))
    //   .andThen(new SetToRGB(ring, 255, 0, 0))
    //   .andThen(Commands.waitSeconds(0.5))
    //   .andThen(new SetToRGB(ring, 0, 255, 0))
    //   .andThen(Commands.waitSeconds(0.5));

    // Keep doing those patterns, one after the other    
    new RepeatCommand(new SequentialCommandGroup(roll, roll2, blink)).schedule();
    // .. or ...
    // roll.andThen(roll2).andThen(blink).repeatedly().schedule();
  }

  @Override
  public void autonomousInit()
  {
    Command fill = new Fill(ring);
    Command rainbow = new Rainbow(ring);

    fill.withTimeout(3.0)
        .andThen(rainbow.withTimeout(5.0))
        .repeatedly().schedule();
  }
}
