// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        SmartDashboard.putBoolean("Arm At Setpoint", arm.atDesiredAngle());
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
        double voltage = -12.0 * RobotOI.joystick.getRightY();
        arm.setVoltage(voltage);
        RobotOI.joystick.setRumble(RumbleType.kBothRumble, Math.abs(voltage/12));
        SmartDashboard.putNumber("Voltage", voltage);
    }

    @Override
    public void autonomousPeriodic()
    {
        // Toggle between two setpoints every 5 secs
        final double setpoint = (System.currentTimeMillis() / 5000) % 2 == 0
                              ? 45.0
                              : 70.0;
        arm.setAngle(setpoint);
        double voltage = arm.hold();
        SmartDashboard.putNumber("Voltage", voltage);
    }
}
