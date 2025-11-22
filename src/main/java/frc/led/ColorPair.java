// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/** Show two colors which roll around the ring */
public class ColorPair extends Command
{
  private final LEDRing ring;
  private final Color one, other;

  public ColorPair(LEDRing ring, Color one, Color other)
  {
    this.ring = ring;
    this.one = one;
    this.other = other;
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
    // Step the 'start' LED every 200 ms
    int start = (int) ((System.currentTimeMillis()  / 200) % LEDRing.N);

    for (int i=0; i<LEDRing.N; ++i)
    {
      int index = (start + i) % LEDRing.N;
      // First half of the LEDs use one color, rest the other
      if (i < LEDRing.N/2)
        ring.buffer.setLED(index, one);
      else
        ring.buffer.setLED(index, other);
    }

    ring.led.setData(ring.buffer);
  }  
}
