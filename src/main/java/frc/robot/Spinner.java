// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Spinner that ejects "fuel" balls */
public class Spinner extends SubsystemBase
{
    private static final double SPINNER_ROTATIONS_PER_MOTOR_TURN = 1.0;

    private final TalonFX motor = new TalonFX(RobotMap.SPINNER);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);
    private final PIDController pid = new PIDController(0, 0, 0);

    private NetworkTableEntry nt_kv = SmartDashboard.getEntry("SpinnerKV");
    private NetworkTableEntry nt_rpm = SmartDashboard.getEntry("SpinnerRPM");
    private NetworkTableEntry nt_setpoint = SmartDashboard.getEntry("SpinnerSetpoint");

    public Spinner()
    {
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
                                                     .withNeutralMode(NeutralModeValue.Coast))
            .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.3));
        motor.getConfigurator().apply(config);
        motor.clearStickyFaults();

        // PID can simply be placed on dashboard
        SmartDashboard.putData("SpinnerPID", pid);
        // For FF, we need to publish and read desired parameters
        nt_kv.setDefaultDouble(0.1);
        nt_setpoint.setDefaultDouble(60);
    }

    @Override
    public void periodic()
    {
        nt_rpm.setNumber(getRPM());
    }

    /** @return Position of spinner in turns of spinner wheel */
    public double getTurns()
    {
        return motor.getPosition().getValueAsDouble() * SPINNER_ROTATIONS_PER_MOTOR_TURN;
    }

    /** @return Spinner speed in rev per minute */
    public double getRPM()
    {
        return motor.getVelocity().getValueAsDouble()
               * SPINNER_ROTATIONS_PER_MOTOR_TURN
               * 60.0;
    }

    /** @param voltage Voltage, should be positive to eject */
    public void setVoltage(double voltage)
    {
        motor.setVoltage(voltage);
    }

    /** Run spinner at setpoint rev per minute */
    public void runAtSpeedSetpoint()
    {
        feedforward.setKv(nt_kv.getDouble(0.0));

        double desired_rpm = nt_setpoint.getDouble(0.0);
        double current_rpm = getRPM();
        double voltage = feedforward.calculateWithVelocities(current_rpm, desired_rpm)
                       + pid.calculate(current_rpm, desired_rpm);
        setVoltage(voltage);
    }
}
