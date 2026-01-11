// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.swervelib.SwerveOI;
import frc.tools.CommandRobotBase;

/** Spinner test
 *
 *  While idle, check "SpinnerPosition" on dashboard.
 *  Does it increment when spinner is manually rotated to "eject"?
 *  If not, invert motor.
 *  Calibrate Spinner.SPINNER_ROTATIONS_PER_MOTOR_TURN
 *
 *  In teleop, verify that spinner 'ejects' when stick is pushed forward.
 *  Determine idea for "SpinnerKV" from "SpinnerVoltage" and "SpinnerRPM".
 *  Find a useful "SpinnerRPM" and make that the default "SpinnerSetpoint".
 *
 *  In auto, configure "SpinnerKV", "SpinnerPID" to reach "SpinnerSetpoint".
 */
public class SpinnerTestRobot extends CommandRobotBase
{
    private final Spinner spinner = new Spinner();

    @Override
    public void robotPeriodic()
    {
        super.robotPeriodic();
        SmartDashboard.putNumber("SpinnerPosition", spinner.getTurns());
    }

    @Override
    public void teleopPeriodic()
    {
        // Send 0-12V, positive when stick is pushed forward
        double voltage = -12.0 * SwerveOI.joystick.getRightY();
        SmartDashboard.putNumber("SpinnerVoltage", voltage);
        spinner.setVoltage(voltage);
    }

    @Override
    public void autonomousPeriodic()
    {
        spinner.runAtSpeedSetpoint();
    }
}
