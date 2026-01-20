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
    private TalonFX motor = MotorHelper.createTalonFX(RobotMap.INTAKE_ARM, false, true, 0.3);

    /** Calibration: Degrees of arm per motor rotation */
    private final static double DEG_PER_ROT = 1.0;

    /** Zero degrees = arm horizontally out */
    private static double ZERO_OFFSET = 0;

        /** Angle when arm is up, intake "in" */
    private static double UP_ANGLE = 0;

    /** Angle when arm is down, intake "out" */
    private static double DOWN_ANGLE = 0;

    // Rotate at a max speed of 45 deg/sec
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(45, 45);
    private ProfiledPIDController pid = new ProfiledPIDController(0.3, 0.03, 0.01, constraints);
    private NetworkTableEntry nt_angle = SmartDashboard.getEntry("Arm Angle"),
                              nt_kg = SmartDashboard.getEntry("Arm kg"),
                              nt_desired_angle = SmartDashboard.getEntry("Set Arm Angle");

    public Arm()
    {
        nt_kg.setDefaultDouble(0.25);
        nt_desired_angle.setDefaultDouble(55);

        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(1.0);
        SmartDashboard.putData("Arm PID", pid);
    }

    public void reset()
    {
        ZERO_OFFSET = getAngle() - UP_ANGLE;
        pid.reset(getAngle());
    }

    /** @return -180..180 degrees */
    public double getAngle()
    {
        double angle = motor.getPosition().getValueAsDouble() * DEG_PER_ROT;
        angle = Math.IEEEremainder(angle - ZERO_OFFSET + UP_ANGLE, 360.0);
        nt_angle.setDouble(angle);
        return angle;
    }

    public void setAngle(double degrees)
    {
        nt_desired_angle.setNumber(degrees);
    }

    public boolean atDesiredAngle()
    {
        return pid.atGoal();
    }

    public void setVoltage(double voltage)
    {
        motor.setVoltage(voltage);
    }

    public void hold()
    {
        final double angle = getAngle();

        double kg = nt_kg.getDouble(0.25);
        double setpoint = MathUtil.clamp(nt_desired_angle.getDouble(50), DOWN_ANGLE, UP_ANGLE);

        double voltage = kg * Math.cos(Math.toRadians(angle))
                       + pid.calculate(angle, setpoint);
        setVoltage(voltage);
    }
}