// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Intake
 *
 *  Opens and closes (via 'Arm')
 *  and has motor to move/pull game pieces in
 */
public class Intake
{
    private final Arm arm = new Arm();

    // 30 Amp limit was insufficient
    private final TalonFX mover = MotorHelper.createTalonFX(RobotMap.INTAKE_MOVER, false, true, 2, 70);
    private final NetworkTableEntry nt_volt_set = SmartDashboard.getEntry("IntakeVoltageSet");
    private final NetworkTableEntry nt_open_angle = SmartDashboard.getEntry("IntakeOpenAngle");
    private final NetworkTableEntry nt_closed_angle = SmartDashboard.getEntry("IntakeClosedAngle");

    public Intake()
    {
        nt_volt_set.setDefaultDouble(9.0);
        nt_open_angle.setDefaultDouble(0.0);
        nt_closed_angle.setDefaultDouble(120.0);
    }

    public double getAngle()
    {
        return arm.getAngle();
    }

    public void open(boolean yes_no)
    {
        open(yes_no, false);
    }

    public void open(boolean yes_no, boolean force_run)
    {
        arm.setAngle(yes_no ? nt_open_angle.getDouble(100) : nt_closed_angle.getDouble(100));
        arm.hold();

        // Turn on when arm is low enough
        boolean auto_run = arm.getAngle() < 20;
        mover.setVoltage((auto_run ||force_run)? nt_volt_set.getDouble(0) : 0);
    }
}
