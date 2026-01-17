// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Handle intake */
public class Intake
{
    // XXX For now DO, will turn into solenoid or motor
    private final DigitalOutput open_intake = new DigitalOutput(RobotMap.INTAKE_OPENER);
    private final TalonFX mover = MotorHelper.createTalonFX(RobotMap.INTAKE_MOVER, false, true, 0.3);
    private final NetworkTableEntry nt_volt_set = SmartDashboard.getEntry("IntakeVoltageSet");

    public Intake()
    {
        nt_volt_set.setDefaultDouble(10.0);
    }

    public void open(boolean yes_no)
    {
        open_intake.set(yes_no);
        mover.setVoltage(yes_no ? nt_volt_set.getDouble(0) : 0);
    }
}
