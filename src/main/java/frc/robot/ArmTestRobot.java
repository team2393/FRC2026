// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;

/** Test the intake arm
 *
 *  1) Find DEG_PER_ROT
 *  2) Find DOWN_ANGLE, UP_ANGLE,
 *     make reset() work
 *  3) Does stick "forward" in teleop move the arm "up"?
 *  4) Tune kg, PID in auto
 */
public class ArmTestRobot extends CommandRobotBase
{
    private final Arm arm = new Arm();
    private final NetworkTableEntry nt_at_setpoint = SmartDashboard.getEntry("ArmAtSetpoint");
    private final NetworkTableEntry nt_setpoint1 = SmartDashboard.getEntry("Setpoint1");
    private final NetworkTableEntry nt_setpoint2 = SmartDashboard.getEntry("Setpoint2");
    private final NetworkTableEntry nt_max_voltage = SmartDashboard.getEntry("MaxVoltage");
    private final NetworkTableEntry nt_voltage = SmartDashboard.getEntry("Voltage");

    public ArmTestRobot()
    {
        nt_setpoint1.setDefaultDouble(45);
        nt_setpoint2.setDefaultDouble(70);
        nt_max_voltage.setDefaultDouble(5);
    }

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        nt_at_setpoint.setBoolean(arm.atDesiredAngle());
    }

    @Override
    public void teleopInit()
    {
        arm.reset();
    }

    @Override
    public void teleopPeriodic()
    {
        arm.getAngle(); // getting angle will show it in NT
        double voltage = -RobotOI.joystick.getRightY() * nt_max_voltage.getDouble(12.0);
        arm.setVoltage(voltage);
        RobotOI.joystick.setRumble(RumbleType.kBothRumble, Math.abs(voltage/12));
        nt_voltage.setDouble(voltage);
    }

    @Override
    public void autonomousInit()
    {
        arm.resetPID();
    }

    @Override
    public void autonomousPeriodic()
    {
        // Toggle between two setpoints every 5 secs
        final double setpoint = (System.currentTimeMillis() / 5000) % 2 == 0
                              ? nt_setpoint1.getDouble(45)
                              : nt_setpoint2.getDouble(45);
        arm.setAngle(setpoint);
        double voltage = arm.hold();
        nt_voltage.setDouble(voltage);
    }
}
