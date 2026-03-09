// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.tools;

/** Helper for dealing with ranges */
public class RangeUtil
{
    /** @param coord Position to check
     *  @param low Low end of range
     *  @param high High end of range
     *  @return Is coordinate within the range?
     */
    public static boolean isBetween(double coord, double low, double high)
    {
        if (low > high)
            throw new RuntimeException("Invalid range");
        return low <= coord  &&  coord <= high;
    }
}
