// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/** "Fill" the LED ring from the bottom
 *
 * LEDs in ring are arranged like this:
 *
 *    02 03
 *  01     04
 * 00       05
 * 11       06
 *  10     07
 *    09 08
 *
 * To "fill", we light up LEDs 08, then 07, 06, ... up to 03
 * on the right half of the ring,
 * in combination with LEDs 09, 10, 11, ... up to 02 on the left half.
 *
 *    02 03
 *  01     04
 * 00       05
 * 11       06
 *  10     07
 *    ## ##
 *
 *    02 03
 *  01     04
 * 00       05
 * 11       06
 *  ##     ##
 *    ## ##
 *     ...
 *    02 03
 *  ##     ##
 * ##       ##
 * ##       ##
 *  ##     ##
 *    ## ##
 *
 *    ## ##
 *  ##     ##
 * ##       ##
 * ##       ##
 *  ##     ##
 *    ## ##
 */
public class Fill extends Command
{
  private final LEDRing ring;

  public Fill(LEDRing the_ring)
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
    // Fill level, 0 to 5, advancing every 200 ms
    int level = (int) ((System.currentTimeMillis()  / 200) % (LEDRing.N/2));
    
    // Scale color from darker to brighter purple
    Color color = new Color((int) MathUtil.interpolate(10, 200, level/5.0),
                            (int) MathUtil.interpolate( 5, 100, level/5.0),
                            (int) MathUtil.interpolate(10, 200, level/5.0));
    ring.clear();
    for (int i=0; i<=level; ++i)
    {
      int right = 8 - i;
      int left = (9 + i) % LEDRing.N;
      ring.buffer.setLED(right, color);
      ring.buffer.setLED(left,  color);
    }
    ring.led.setData(ring.buffer);
  }  
}
