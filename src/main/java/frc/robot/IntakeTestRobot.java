// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;

/** Test the intake */
public class IntakeTestRobot extends CommandRobotBase
{
    private final PowerDistribution power_dist = new PowerDistribution();
    private final TalonFX intake_mover = MotorHelper.createTalonFX(RobotMap.INTAKE_MOVER, false, true, 0.3);
    private final NetworkTableEntry nt_volt_set = SmartDashboard.getEntry("IntakeVoltageSet");

    public IntakeTestRobot()
    {
        power_dist.clearStickyFaults();
        SmartDashboard.putData("Power", power_dist);
        nt_volt_set.setDefaultDouble(10);
    }

    void setVoltage(double voltage)
    {
        intake_mover.setVoltage(voltage);
        SmartDashboard.putNumber("IntakeVoltage", voltage);
    }

    @Override
    public void teleopPeriodic()
    {
        setVoltage(-12*RobotOI.joystick.getRightY());
    }

    @Override
    public void autonomousPeriodic()
    {
        setVoltage(nt_volt_set.getDouble(0));
    }
}
