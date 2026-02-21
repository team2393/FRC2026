// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;

/** Storage test */
public class StorageTestRobot extends CommandRobotBase
{
    // At 5V, mechanism uses ~20 amp. With balls, it runs up to ~30
    private final TalonFX motor = MotorHelper.createTalonFX(9, false, false, 0, 30.0);

    @Override
    public void teleopPeriodic()
    {
        double voltage = -12*RobotOI.joystick.getRightY();
        SmartDashboard.putNumber("Voltage", voltage);
        motor.setVoltage(voltage);
    }
}
