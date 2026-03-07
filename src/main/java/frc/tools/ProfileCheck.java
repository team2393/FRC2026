// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.tools;

/** Check motion profile settings */
public class ProfileCheck
{
    public static void main(String[] args)
    {
        // Motion profile
        double max_speed = 45;
        double accel = 45;

        // Distance to cover
        double dist = 120-5;

        // Time to accelerate, distance covered in that time
        double t1 = max_speed/accel;
        double d1 = 0.5*accel*t1*t1;

        // Deceleration will be the same
        double t3 = t1;
        double d3 = d1;

        // Remaining distance, covered at full speed
        double d2 = dist-d1-d3;
        double t2 = d2/max_speed;

        if (d1+d3 > dist)
        {
            // Can't accelerate to full speed
            // Accelerate half the distane, decellerate the other half
            d1 = d3 = dist/2;
            d2 = 0;
            t1 = t3 = Math.sqrt(2*d1/accel);
            t2 = 0;
        }

        System.out.format("It will take %.1f s to accelerate, moving by %.1f\n", t1, d1);
        System.out.format("We'll run for %.1f s at full speed, covering %.1f\n", t2, d2);
        System.out.format("It will take %.1f s to decellerate, moving by %.1f\n",  t3, d3);

        double t = t1+t2+t3;
        System.out.format("In total, it will take %.1f sec to move %.1f\n", t, dist);
    }
}
