// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Storage */
public class Storage
{
    // At 5V, mechanism uses ~20 amp. With balls, it runs up to ~30
    private final TalonFX motor = MotorHelper.createTalonFX(RobotMap.STOREAGE_MOVER, false, false, 0, 30.0);
    private final NetworkTableEntry nt_storage_voltage = SmartDashboard.getEntry("StorageVoltage");

    public Storage()
    {
        nt_storage_voltage.setDefaultDouble(8.0);
    }

    public void run(boolean on_off)
    {
        motor.setVoltage(on_off ? nt_storage_voltage.getDouble(0)
                                : 0);
    }
}
