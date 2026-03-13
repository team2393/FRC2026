// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.tools.CommandRobotBase;

/** Spinner test */
public class CTREVelocityControlTestRobot extends CommandRobotBase
{
    private final static double UNITS_PER_TURN = 1.0;

    private final TalonFX motor = MotorHelper.createTalonFX(RobotMap.SPINNER, false, false, 0,40);
    private final TalonFX motor2 = MotorHelper.createTalonFX(RobotMap.SPINNER2, false, false, 0, 40);


    private NetworkTableEntry nt_setpoint  = SmartDashboard.getEntry("Setpoint"),
                              nt_setpoint1 = SmartDashboard.getEntry("Setpoint1"),
                              nt_setpoint2 = SmartDashboard.getEntry("Setpoint2"),
                              nt_velocity  = SmartDashboard.getEntry("Velocity");

    public CTREVelocityControlTestRobot()
    {
        motor2.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));

        var pid = new Slot0Configs();
        pid.kS = 0.1;
        pid.kV = 0.117;
        pid.kP = 0.2;
        pid.kI = 0.1;
        pid.kD = 0;
        motor.getConfigurator().apply(pid);

        nt_setpoint.setDefaultDouble(0);
        nt_setpoint1.setDefaultDouble(1800.0/60.0);
        nt_setpoint2.setDefaultDouble(2250.0/60.0);
    }

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        nt_velocity.setDouble(motor.getVelocity().getValueAsDouble() * UNITS_PER_TURN);
    }

    @Override
    public void teleopPeriodic()
    {
        motor.setVoltage(-12.0 * RobotOI.joystick.getRightY());
    }

    private VelocityVoltage request = new VelocityVoltage(0);

    @Override
    public void autonomousPeriodic()
    {
        double setpoint = (System.currentTimeMillis() / 5000) % 2 == 0
                        ? nt_setpoint1.getDouble(0)
                        : nt_setpoint2.getDouble(0);
        nt_setpoint.setDouble(setpoint);
        request.withVelocity(setpoint / UNITS_PER_TURN);
        motor.setControl(request);
    }
}
