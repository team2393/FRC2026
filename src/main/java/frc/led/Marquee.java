// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/** "Marquee" pattern */
public class Marquee extends Command
{
  private final LEDRing ring;

  public Marquee(LEDRing the_ring)
  {
    ring = the_ring;
    addRequirements(ring);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  @Override
  public void execute()
  {
    // Start LED (0..N-1)
    int start = ((int) (System.currentTimeMillis()/200) % LEDRing.N);

    ring.clear();
    for (int i=0; i<LEDRing.N; ++i)
    {
      if ( ((i-start)/2) % 2 == 0)
        ring.buffer.setLED(i,  Color.kRed);
      else
        ring.buffer.setLED(i,  Color.kGreen);
    }
    ring.led.setData(ring.buffer);
  }  
}
