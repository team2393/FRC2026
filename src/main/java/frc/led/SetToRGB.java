// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.led;

import edu.wpi.first.wpilibj2.command.Command;

/** Command that sets all colors of LEDRing to the same color and is then finished */
public class SetToRGB extends Command
{
  private final LEDRing ring;
  private final int red, green, blue;

  /** @param the_ring LEDRing to use
   *  @param red   0 to 255
   *  @param green 0 to 255
   *  @param blue  0 to 255
   */
  public SetToRGB(LEDRing the_ring, int red, int green, int blue)
  {
    ring = the_ring; 
    this.red = red;
    this.green = green;
    this.blue = blue;
    addRequirements(ring);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    for (int i=0;  i<LEDRing.N;  ++i)
      ring.buffer.setRGB(i, red, green, blue);
  
    ring.led.setData(ring.buffer);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
