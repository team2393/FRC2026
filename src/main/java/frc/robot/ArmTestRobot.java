// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.tools.CommandRobotBase;

/** Test the intake arm
 *
 *  1) Find DEG_PER_ROT
 *  2) Find DOWN_ANGLE, UP_ANGLE,
 *     make reset() work
 *  3) Does "forward" move up in teleop?
 *  4) Tune kg, PID in auto
 */
public class ArmTestRobot extends CommandRobotBase
{
    private final Arm arm = new Arm();

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
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
        arm.setVoltage(-12.0 * RobotOI.joystick.getRightY());
    }

    @Override
    public void autonomousPeriodic()
    {
        arm.hold();
    }
}
