// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demo;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;
import frc.tools.KeepOnFilter;

/** Digital filter demo */
public class DigitalFilterDemoRobot extends CommandRobotBase
{
    // Detect when raw signal stays on for at least 0.2 secs
    private final Debouncer debounce = new Debouncer(0.2);
    // When signal turns off, keep it on for another 0.3 s
    private final KeepOnFilter keep = new KeepOnFilter(0.3);
    // Delay on/off by 0.1 sec
    private final Debouncer delay = new Debouncer(0.1, DebounceType.kBoth);
    private final NetworkTableEntry nt_raw = SmartDashboard.getEntry("raw");
    private final NetworkTableEntry nt_debounced = SmartDashboard.getEntry("debounced");
    private final NetworkTableEntry nt_keep = SmartDashboard.getEntry("keep");
    private final NetworkTableEntry nt_delay = SmartDashboard.getEntry("delay");

    @Override
    public void autonomousPeriodic()
    {
        // Change every 500 ms
        boolean raw = (System.currentTimeMillis() / 500) % 2 == 1;

        boolean debounced = debounce.calculate(raw);
        boolean kept = keep.calculate(debounced);
        boolean delayed = delay.calculate(kept);
        nt_raw.setBoolean(raw);
        nt_debounced.setBoolean(debounced);
        nt_keep.setBoolean(kept);
        nt_delay.setBoolean(delayed);
    }
}
