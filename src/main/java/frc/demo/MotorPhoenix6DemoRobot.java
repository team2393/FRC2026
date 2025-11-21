// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demo;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.tools.CommandRobotBase;

/** Motor Demo
 *  
 *  Runs motor with the more recent Phoennix 6 API
 *  while still accessing the older Pigeon with Phoenix v5 API.
 */
public class MotorPhoenix6DemoRobot extends CommandRobotBase
{
    private final CommandXboxController joystick = new CommandXboxController(0);
    
    // Move motor by hand, note distance in meters over indicated turns 
    private final static double METERS_PER_TURN = 0.3/6.3;

    // Motor using Phoenix 6 library
    private final TalonFX motor = new TalonFX(1);

    // Older pigeon still supported by Phoenix 5
    private final PigeonIMU gyro = new PigeonIMU(0);

    public MotorPhoenix6DemoRobot()
    {
        // Some settings are only available via "...Configuration" class
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.3));
        motor.getConfigurator().apply(config);
        
        // Others via methods
        motor.clearStickyFaults();
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        double turns = motor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Turns", turns);
        SmartDashboard.putNumber("Position", turns * METERS_PER_TURN);
        SmartDashboard.putNumber("Speed", motor.getVelocity().getValueAsDouble() * METERS_PER_TURN);
        SmartDashboard.putNumber("Heading", gyro.getFusedHeading());
    }    

    @Override
    public void teleopPeriodic()
    {
        motor.setVoltage(joystick.getRightY()*12.0);
    }
}
