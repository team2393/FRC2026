// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.tools;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;

/** Lookup Table
 *
 *  Performs linear interpolation between values in table
 */
public class LookupTable
{
    /** One table entry */
    public static record Entry(double distance, double speed, double hood, double deviation)
    {
        @Override
        public String toString()
        {
            return String.format("Distance %5.2f: Speed %.1f, Hood %.1f, Deviation %.1f",
                                 distance, speed, hood, deviation);
        }
    }

    /** Table of data points */
    private final List<Entry> table = new ArrayList<>();

    public LookupTable(final double... cell)
    {
        if (cell.length % 4 != 0)
            throw new IllegalArgumentException("Need list of { distance, speed, hood, deviation}, {distance, speed, ...");
        for (int i = 0;  i < cell.length;  i += 4)
            table.add(new Entry(cell[i], cell[i+1], cell[i+2], cell[i+3]));
        // Table must be sorted by distance
        table.sort((a, b) -> Double.compare(a.distance, b.distance));
    }

    /** @param distance Distance
     *  @return Settings for that distance
     */
    public Entry lookup(final double distance)
    {
        final int n = table.size();
        // Is distance outside of table's distance range?
        if (distance <= table.get(0).distance)
            return table.get(0);
        if (distance >= table.get(n-1).distance)
            return table.get(n-1);
        // Binary search starting with left, right set to complete table
        // https://en.wikipedia.org/wiki/Binary_search_algorithm#Procedure_for_finding_the_leftmost_element
        int l = 0, r = n;
        while (l < r)
        {   // Binary search: Find middle index, rounding down(!)
            final int m = (l + r) / 2;
            if (table.get(m).distance < distance)
                l = m+1;   // distance must be in upper half
            else
                r = m;     // distance must be in lower half (or exact match)
        }
        // For an exact match, [l] is that element
        if (table.get(l).distance == distance)
            return table.get(l);
        // Otherwise l points to the next larger element,
        // so distance is between element [l-1] and [l].
        // Interpolate between those two points
        Entry start = table.get(l-1), end = table.get(l);
        double fraction = (distance     - start.distance)
                        / (end.distance - start.distance);
        return new Entry(distance,
                         MathUtil.interpolate(start.speed,     end.speed,     fraction),
                         MathUtil.interpolate(start.hood,      end.hood,      fraction),
                         MathUtil.interpolate(start.deviation, end.deviation, fraction));
    }

    // Test/demo
    public static void main(String[] args)
    {
        // Example for lookup of spinner speed, hood angle, deviation for distance
        final LookupTable settings = new LookupTable(30, 65, 0, 0,
                                                     20, 60, 0, 0,
                                                      0, 55, 0, 0,
                                                    -20, 60, 0, 0,
                                                    -30, 75, 0, 0);

        for (double d : new double[] { 40, 30, 25, 20, 10, 5, 0, -5, -10, -20, -30, -40})
            System.out.println(d + " -> " + settings.lookup(d));
    }
}