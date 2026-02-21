// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Hood that controls ball ejection angle */
public class Hood extends SubsystemBase
{
    /** Encoder steps per percent movement */
    private final double STEPS_PER_PERC = 1.0; // 2048 * 14.0 / 100.0;

    /** Position is limited to 0 .. max [percent] */
    private final double MIN_POS = 0, MAX_POS = 100.0;

    private final TalonFX hood = MotorHelper.createTalonFX(RobotMap.HOOD, true, false, 0.3);

    private final NetworkTableEntry nt_setpoint = SmartDashboard.getEntry("HoodSetpoint");
    private final NetworkTableEntry nt_position = SmartDashboard.getEntry("Hood");

    /** Maximum speed [mm/s] */
    // About half the actual max speed is a good setting
    private final double MAX_PERC_PER_SEC = 200.0;
    // TODO Profiled PID controller
    // private final ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0,
                            // new TrapezoidProfile.Constraints(MAX_PERC_PER_SEC, MAX_PERC_PER_SEC));
    private final PIDController pid = new PIDController(0, 0, 0);

    private double zero_offset = 0.0;

    public Hood()
    {
        nt_setpoint.setDefaultDouble(-1.0);
        SmartDashboard.putData("HoodPID", pid);
    }

    /** Reset position encoder
     *
     *  Must be called when robot starts up,
     *  assuming hood is at bottom position
     */
    public void reset()
    {
        zero_offset = hood.getPosition().getValueAsDouble();
    }

    /** @return Position in 0-100 percent */
    public double getPosition()
    {
        return (hood.getPosition().getValueAsDouble() - zero_offset) / STEPS_PER_PERC;
    }

    /** @param voltage Motor voltage -12..+12 */
    public void setVoltage(final double voltage)
    {
        hood.setVoltage(voltage);
    }

    private void holdPosition()
    {
        double setpoint = nt_setpoint.getDouble(0.0);
        if (setpoint > 0)
        {
            double voltage = pid.calculate(getPosition(), MathUtil.clamp(setpoint, MIN_POS, MAX_POS));
            voltage = MathUtil.clamp(voltage, -6, 6);
            setVoltage(voltage);
        }
        else
            setVoltage(0);
    }

    @Override
    public void periodic()
    {
        holdPosition();
        nt_position.setDouble(getPosition());
    }
}