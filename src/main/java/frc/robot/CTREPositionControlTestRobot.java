// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;

/** Storage or Feeder test */
public class CTREPositionControlTestRobot extends CommandRobotBase
{
    private final static double UNITS_PER_TURN = 0.3/6.3;

    private final TalonFX motor = MotorHelper.createTalonFX(1, false, false, 0, 0.0);

    private NetworkTableEntry nt_setpoint = SmartDashboard.getEntry("Setpoint"),
                              nt_position = SmartDashboard.getEntry("Position");

    public CTREPositionControlTestRobot()
    {
        var pid = new Slot0Configs();
        pid.kS = 0;
        pid.kP = 0;
        pid.kI = 0;
        pid.kD = 0;
        motor.getConfigurator().apply(pid);

        // var mm = new MotionMagicConfigs()
        //         .withMotionMagicAcceleration(1)
        //         .withMotionMagicCruiseVelocity(1);
        // motor.getConfigurator().apply(mm);

        nt_setpoint.setDefaultDouble(0);
    }

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        nt_position.setDouble(motor.getVelocity().getValueAsDouble() * UNITS_PER_TURN);
    }

    @Override
    public void teleopPeriodic()
    {
        motor.setVoltage(-12.0 * RobotOI.joystick.getRightY());
    }

    private final PositionVoltage request = new PositionVoltage(0);
    // private final MotionMagicVoltage request = new MotionMagicVoltage(0);

    @Override
    public void autonomousPeriodic()
    {
        request.withPosition(nt_setpoint.getDouble(0) / UNITS_PER_TURN);
        motor.setControl(request);
    }
}
