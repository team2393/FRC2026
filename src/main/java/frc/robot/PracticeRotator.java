// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.swervelib.RotatorBase;

/** Rotator using Kraken and CANcoder */
public class PracticeRotator extends RotatorBase
{
    private final double DEG_PER_TURN = 360.0;
    private final TalonFX motor;
    private final CANcoder encoder;

    /** Construct Rotator
     *  @param index Rotator index 0..3
     *  @param motor_id Motor CAN id
     *  @param encoder_id Encoder CAN id
     *  @param offset Offset from 'forward' in degrees
     */
    public PracticeRotator(int index, int motor_id, int encoder_id, double offset)
    {
        super(index, offset, 0.0, 0.05, 0.5, 0.001, 10.0);
        motor = MotorHelper.createTalonFX(motor_id, false, false, 0.1);

        encoder = new CANcoder(encoder_id);
        encoder.clearStickyFaults();
        CANcoderConfiguration configs = new CANcoderConfiguration();
        // Default range: -0.5 .. 0.5 turns, CCW
        configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5)
                            .withMagnetOffset(0.0)
                            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        encoder.getConfigurator().apply(configs);
    }

    @Override
    public double getRawDegrees()
    {
        return encoder.getAbsolutePosition().getValueAsDouble() * DEG_PER_TURN;
    }

    @Override
    public void setVoltage(double voltage)
    {
        motor.setVoltage(voltage);
    }
}
