// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Intake Arm: Motor to rotate out/in
 *
 *  Robot must power up with arm all up=in
 */
public class Arm
{
    private final TalonFX motor = MotorHelper.createTalonFX(RobotMap.INTAKE_ARM, false, true, 0.3);

    /** Calibration: Degrees of arm per motor rotation */
    private final static double DEG_PER_ROT = 1.0;

    /** Angle when arm is up, intake "in" */
    private final static double UP_ANGLE = 0;

    /** Angle when arm is down, intake "out" */
    private final static double DOWN_ANGLE = 0;

    // Rotate at a max speed of 45 deg/sec
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(45, 45);
    private final ProfiledPIDController pid = new ProfiledPIDController(0.0, 0.0, 0.0, constraints);
    private final NetworkTableEntry nt_angle = SmartDashboard.getEntry("Arm Angle"),
                                    nt_kg = SmartDashboard.getEntry("Arm kg"),
                                    nt_desired_angle = SmartDashboard.getEntry("Set Arm Angle");

    /** Zero degrees = arm horizontally out */
    private static double ZERO_OFFSET = 0;

    public Arm()
    {
        nt_kg.setDefaultDouble(0.0);
        nt_desired_angle.setDefaultDouble(55);

        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(1.0);
        SmartDashboard.putData("Arm PID", pid);
    }

    /** @return -180..180 degrees */
    private double getRawAngle()
    {
        return motor.getPosition().getValueAsDouble() * DEG_PER_ROT;
    }

    /** Reset angle, assuming arm is positioned at UP_ANGLE */
    public void reset()
    {
        // We are at UP_ANGLE.
        // Assume raw angle reports 30
        // -> ZERO_OFFSET = 30 - UP_ANGLE
        ZERO_OFFSET = getRawAngle() - UP_ANGLE;
        // getAngle() will now return
        // 30 - ZERO_OFFSET =
        // 30 - (30 - UP_ANGLE) = UP_ANGLE
        pid.reset(getAngle());
    }

    /** @return -180..180 degrees, zero means straight forward, 90 vertical */
    public double getAngle()
    {
        final double angle = Math.IEEEremainder(getRawAngle() - ZERO_OFFSET, 360.0);
        nt_angle.setDouble(angle);
        return angle;
    }

    /** @param degrees Desired angle */
    public void setAngle(final double degrees)
    {
        nt_desired_angle.setNumber(degrees);
    }

    /** @return Is arm at desired angle? */
    public boolean atDesiredAngle()
    {
        return pid.atGoal();
    }

    /** @param voltage Direct drive voltage */
    public void setVoltage(final double voltage)
    {
        motor.setVoltage(voltage);
    }

    /** Hold at desired angle */
    public void hold()
    {
        final double angle = getAngle();

        final double kg = nt_kg.getDouble(0.25);
        final double setpoint = MathUtil.clamp(nt_desired_angle.getDouble(50), DOWN_ANGLE, UP_ANGLE);

        final double voltage = kg * Math.cos(Math.toRadians(angle))
                             + pid.calculate(angle, setpoint);
        setVoltage(voltage);
    }
}