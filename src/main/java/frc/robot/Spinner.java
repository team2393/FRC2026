// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Spinner that ejects "fuel" balls */
public class Spinner extends SubsystemBase
{
    /** How much does the spinner rotate for one motor turn?
     *  To calibrate, start with 1.0
     */
    private static final double SPINNER_ROTATIONS_PER_MOTOR_TURN = 1.0;

    /** Which RPM error do we consider 'close enough' to the setpoint? */
    private static final double ACCEPTED_RPM_ERROR = 10;

    /** Motor that's controlled */
    private final TalonFX motor = MotorHelper.createTalonFX(RobotMap.SPINNER, false, false, 0.3);

    /** Motor that follows the primary motor*/
    // private final TalonFX motor2 = MotorHelper.createTalonFX(RobotMap.SPINNER2, false, false, 0.3);

    /** How much voltage do we need for desired RPM? */
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);

    /** React to disturbances */
    private final PIDController pid = new PIDController(0, 0, 0);

    /** Feed-forward kv */
    private NetworkTableEntry nt_kv = SmartDashboard.getEntry("SpinnerKV");

    /** Current spinner speed */
    private NetworkTableEntry nt_rpm = SmartDashboard.getEntry("SpinnerRPM");

    /** Desired spinner speed */
    private NetworkTableEntry nt_setpoint = SmartDashboard.getEntry("SpinnerSetpoint");

    /** Is speed close enough to desired speed? */
    private NetworkTableEntry nt_at_setpoint = SmartDashboard.getEntry("SpinnerAtSetpoint");

    /** De-bounce "at setpoint" */
    private final Debouncer debouncer = new Debouncer(0.5);

    /** Is speed close enough to desired speed? */
    private boolean is_at_setpoint;

    public Spinner()
    {
        // motor2.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));

        // PID can simply be placed on dashboard
        SmartDashboard.putData("SpinnerPID", pid);
        // For FF, we need to publish and read desired parameters
        nt_kv.setDefaultDouble(0.1);
        nt_setpoint.setDefaultDouble(60);
    }

    @Override
    public void periodic()
    {
        double rpm = getRPM();
        nt_rpm.setNumber(rpm);

        // Are we close enough to desired speed?
        boolean close_to_setpoint = Math.abs(nt_setpoint.getDouble(0.0) - rpm) <= ACCEPTED_RPM_ERROR;
        // .. for a certain time?
        is_at_setpoint = debouncer.calculate(close_to_setpoint);
        nt_at_setpoint.setBoolean(is_at_setpoint);
    }

    /** @return Position of spinner in turns of spinner wheel */
    public double getTurns()
    {
        return motor.getPosition().getValueAsDouble() * SPINNER_ROTATIONS_PER_MOTOR_TURN;
    }

    /** @return Spinner speed in rev per minute */
    public double getRPM()
    {
        return motor.getVelocity().getValueAsDouble()
               * SPINNER_ROTATIONS_PER_MOTOR_TURN
               * 60.0;
    }

    /** @return Has spinner been at setpoint for a little while? */
    public boolean isAtSetpoint()
    {
        return is_at_setpoint;
    }

    // High pass filter shows change in value, any change slower than 0.1 seconds are ignored.
    // This filters out small changes, but rapid drop in current as ball is ejected
    // gets detected.
    private final LinearFilter highpass = LinearFilter.highPass(0.1, TimedRobot.kDefaultPeriod);
    private final NetworkTableEntry nt_current = SmartDashboard.getEntry("SpinnerCurrent");
    private final NetworkTableEntry nt_change = SmartDashboard.getEntry("SpinnerCurrentChange");

    /** @param voltage Voltage, should be positive to eject */
    public void setVoltage(double voltage)
    {
        motor.setVoltage(voltage);

        // XXX Try to detect when a game item is ejected
        double current = motor.getTorqueCurrent().getValueAsDouble();
        double change = highpass.calculate(current);
        nt_current.setNumber(current);
        nt_change.setNumber(change);

        // See https://github.com/team2393/FRC2022/blob/main/src/main/java/frc/robot/cargo/Spinner.java
        // ejected = delay.compute(remember_shot.compute(Math.abs(change) > 10.0));
    }

    /** Run spinner at setpoint rev per minute */
    public void runAtSpeedSetpoint()
    {
        feedforward.setKv(nt_kv.getDouble(0.0));

        double desired_rpm = nt_setpoint.getDouble(0.0);
        double current_rpm = getRPM();
        // Feed forward should get us close to the desired speed,
        // and PID then corrects disturbances
        double voltage = feedforward.calculateWithVelocities(current_rpm, desired_rpm)
                       + pid.calculate(current_rpm, desired_rpm);
        setVoltage(voltage);
    }
}
