// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Helper for dealing with motors */
public class MotorHelper
{
    /** Create motor
     *  @param can_id CAN ID
     *  @param invert Invert direction?
     *  @param brake Brake, or coast?
     *  @param ramp_up_secs How long to delay ramp-up
     *  @return TalonFX
     */
    public static TalonFX createTalonFX(int can_id, final boolean invert, boolean brake, double ramp_up_secs)
    {
        TalonFX motor = new TalonFX(can_id);

        InvertedValue inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        NeutralModeValue neutral = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(inverted)
                                                     .withNeutralMode(neutral))
            .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(ramp_up_secs));
        motor.getConfigurator().apply(config);
        motor.clearStickyFaults();

        return motor;
    }
}
