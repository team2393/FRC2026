// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.led;

import edu.wpi.first.wpilibj2.command.Command;

/** lights up the whole ring which gradually morphs between colors of the rainbow */
public class Rainbow extends Command
{
  private final LEDRing ring;

  public Rainbow(LEDRing ring)
  {
    this.ring = ring;
    addRequirements(ring);
  }
  
  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  // How long should it take for the rainbow 'snake' to move around the ring?
  private final int millisecs_for_one_round = 2000;

  // ms for stepping one LED, so all N are handled in millisecs_for_one_round
  private final int period = millisecs_for_one_round / LEDRing.N;

  @Override
  public void execute()
  {
    // Pixel at head of rainbow 'snake' that moves around the ring.
    int head = (int) ((System.currentTimeMillis()  / period) % LEDRing.N);

    for (int i = 0; i < LEDRing.N; ++i)
    {
      // Position of LED within the 'snake'
      int pos = (i+head) % LEDRing.N;

      // Map pos 0..N to hue 0..180
      int hue = (int) ((pos * 180) / LEDRing.N);
      ring.buffer.setHSV(i, hue, 255, 255);
    }
    ring.led.setData(ring.buffer);
  }  
}
