// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;

/** Storage or Feeder test */
public class StorageOrFeederTestRobot extends CommandRobotBase
{
    // Storage:
    // At 5V, mechanism uses ~20 amp. With balls, it runs up to ~30
    private final TalonFX motor = MotorHelper.createTalonFX(RobotMap.FEEDER, false, false, 0, 30.0);
    private final DigitalInput feeder_sensor = new DigitalInput(RobotMap.FEEDER_SENSOR);

    public boolean haveBall()
    {
        return feeder_sensor.get();
    }

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        SmartDashboard.putBoolean("Have Ball", haveBall());
    }

    @Override
    public void teleopPeriodic()
    {
        double voltage = -12*RobotOI.joystick.getRightY();
        SmartDashboard.putNumber("Voltage", voltage);
        motor.setVoltage(voltage);
    }

    @Override
    public void autonomousPeriodic()
    {
        // 5 V is good to feed until sensor detects ball
        // 6 V causes ball to be very close to spinner or touching
        motor.setVoltage(haveBall() ? 0.0 : 5.0);
    }
}
