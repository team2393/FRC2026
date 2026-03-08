// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.tools;

/** Helper for computing statistics */
public class Stats
{
    /** Number of samples */
    private long n;
    /** Stats */
    private double min, max, avg;

    public Stats()
    {
        reset();
    }

    public void reset()
    {
        n = 0;
        min = Double.NaN;
        max = Double.NaN;
        avg = 0;
    }

    public double getMin()
    {
        return min;
    }

    public double getMax()
    {
        return max;
    }

    public double getAvg()
    {
        return avg;
    }

    public void addSample(double x)
    {
        ++n;

        if (n == 1)
            min = max = x;
        else
        {
            if (x < min)
                min = x;
            if (x > max)
                max = x;
        }

        // Average over n values is defined as
        //     avg_n = sum(x_i, i=1..n) / n
        // but computed that way, the sum can get too large
        // to fit into a variable.
        // https://math.stackexchange.com/questions/106313/regular-average-calculated-accumulatively
        // n * avg_n =  sum(x_i, i=1..n)
        //           =  x_n + sum(x_i, i=1..n-1)   // Note: avg_n-1 = sum(x_i, i=1..n-1) / (n-1)
        //           =  x_n + (n-1) * avg_n-1
        // --> avg_n = (x_n + (n-1) * avg_n-1) / n
        //           = (x_n + n * avg_n-1 - avg_n-1) / n
        //           =  x_n /n  + avg_n-1 - avg_n-1 / n
        //           = (x_n - avg_n-1)/n  + avg_n-1
        avg += (x - avg) / n;

        // n can of course exceed Integer.MAX_VALUE.
        // We defer the problem until Long.MAX_VALUE ...
    }

    @Override
    public String toString()
    {
        return "Min " + min + ", Max " + max + ", Avg " + avg;
    }

    public static void main(String[] args)
    {
        Stats stats = new Stats();
        stats.addSample(-1);
        stats.addSample(1);
        stats.addSample(2);
        stats.addSample(3);
        stats.addSample(4);
        System.out.println(stats);
    }
}
