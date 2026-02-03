// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.swervelib.DriverBase;

/** Driver using Kraken */
public class PracticeDriver extends DriverBase
{
    // Calibrate: Start with 1.0, then determine 'turns' for 1 m (better: 10 m)
    private final static double METERS_PER_TURN = 0.04677860840234989;

    private final TalonFX motor;

    /** @param index Driver index 0..3
     *  @param id CAN id
    */
    public PracticeDriver(int index, int id)
    {
        super(index, 0.05, 2.5, 0.25, 2.0, 0.01);
        motor = MotorHelper.createTalonFX(id, false, true, 0.1);
    }

    protected double getRawPosition()
    {
        return motor.getPosition().getValueAsDouble() * METERS_PER_TURN;
    }

    protected double getRealSpeed()
    {
        return motor.getVelocity().getValueAsDouble() * METERS_PER_TURN;
    }

    public void setVoltage(double voltage)
    {
        motor.setVoltage(voltage);
    }
}
