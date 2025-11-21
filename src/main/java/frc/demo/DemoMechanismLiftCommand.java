// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.demo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DemoMechanismLiftCommand extends Command
{
    private final DemoMechanism mechanism;
    private final double start, end, seconds;
    private final Timer timer = new Timer();

    public DemoMechanismLiftCommand(DemoMechanism mechanism, double start, double end, double seconds)
    {
        this.mechanism = mechanism;
        this.start = start;
        this.end = end;
        this.seconds = seconds;
    }

    @Override
    public void initialize()
    {
        timer.restart();
    }

    @Override
    public void execute()
    {
        mechanism.setLift(MathUtil.interpolate(start, end, timer.get()/seconds));
    }

    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(seconds);
    }
}
