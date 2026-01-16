// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Hood that controls ball ejection angle */
public class Hood extends SubsystemBase
{
    /** Encoder steps per percent movement */
    private final double STEPS_PER_PERC = 2048 * 14.0 / 100.0;

    /** Position is limited to 0 .. max [percent] */
    private final double MAX_POS_PERC = 100.0;


    private final TalonFX hood = MotorHelper.createTalonFX(RobotMap.HOOD, true, true, 0.3);
    private final DigitalInput home = new DigitalInput(RobotMap.HOOD_HOME);

    private final NetworkTableEntry nt_setpoint = SmartDashboard.getEntry("HoodSetpoint");
    private final NetworkTableEntry nt_position = SmartDashboard.getEntry("Hood");
    private final NetworkTableEntry nt_at_home = SmartDashboard.getEntry("HoodHome");

    /** Maximum speed [mm/s] */
    // About half the actual max speed is a good setting
    private final double MAX_PERC_PER_SEC = 200.0;
    // TODO Profiled PID controller
    // private final ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0,
                            // new TrapezoidProfile.Constraints(MAX_PERC_PER_SEC, MAX_PERC_PER_SEC));
    private final PIDController pid = new PIDController(0, 0, 0);

    public Hood()
    {
        nt_setpoint.setDefaultDouble(5.0);
        SmartDashboard.putData("HoodPID", pid);
    }

    /** Reset position encoder */
    public void reset()
    {
        hood.setPosition(0.0);
    }

    public void home()
    {
        // TODO
    }

    /** @return Position in mm */
    public double getPosition()
    {
        return hood.getPosition().getValueAsDouble() / STEPS_PER_PERC;
    }

    public boolean atHome()
    {
        // REV magnetic limit switch contains an internal pull-up resistor,
        // and is 'active low', connecting to ground when detecting the magnet.
        // If we're not at the limit, i.e. magnet is not at the sensor, it reports 'true'
        // because of the pull-up.
        // If we are at the limit, i.e. magnet is close to the sensor, it reports 'false'
        // because of the active low.
        // When we disconnect the cable, the pull-up internal to the RoboRIO reports 'true',
        // so not really fail-safe.
        return ! home.get();
    }

    /** @param voltage Motor voltage -12..+12 */
    public void setVoltage(final double voltage)
    {
        hood.setVoltage(voltage);
    }

    public void holdPosition()
    {
        double voltage = pid.calculate(getPosition(), MathUtil.clamp(nt_setpoint.getDouble(0.0), 0, MAX_POS_PERC));
        voltage = MathUtil.clamp(voltage, -6, 6);
        setVoltage(voltage);
    }

    @Override
    public void periodic()
    {
        nt_position.setDouble(getPosition());
        nt_at_home.setBoolean(atHome());
    }
}